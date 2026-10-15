// W11R1: finite, state-reconciled lifecycle RPCs. No velocity or TF ownership.
// Retains Nav2's 4 s bond / 0.1 s heartbeat contract for localization/navigation.
#include <chrono>
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "bondcpp/bond.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using Get = lifecycle_msgs::srv::GetState;
using Change = lifecycle_msgs::srv::ChangeState;
using Manage = nav2_msgs::srv::ManageLifecycleNodes;

class BoundedManager : public rclcpp::Node
{
public:
  BoundedManager() : Node("bounded_lifecycle_manager")
  {
    names_ = declare_parameter<std::vector<std::string>>("node_names", std::vector<std::string>{});
    autostart_ = declare_parameter("autostart", false);
    if (declare_parameter("bond_timeout", 4.0) != 4.0) {
      throw std::runtime_error("W11R1 requires the frozen 4 s bond timeout");
    }
    declare_parameter("attempt_respawn_reconnection", false);
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rpc_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rpc_executor_.add_callback_group(rpc_group_, get_node_base_interface());
    for (const auto & n : names_) {
      get_[n] = create_client<Get>(n + "/get_state", rclcpp::ServicesQoS(), rpc_group_);
      change_[n] = create_client<Change>(n + "/change_state", rclcpp::ServicesQoS(), rpc_group_);
    }
    stop_ = create_client<std_srvs::srv::Trigger>("/t1/safety/stop");
    service_ = create_service<Manage>(std::string(get_name()) + "/manage_nodes",
      [this](const Manage::Request::SharedPtr request, Manage::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(control_mutex_);
        if (closing_) {response->success = false; return;}
        if (request->command == Manage::Request::STARTUP && !failed_) {
          response->success = active_ || startup();
        } else if (request->command == Manage::Request::SHUTDOWN) {
          stop_and_deactivate();
          response->success = true;
        } else {
          response->success = false;  // No automatic retry/re-arm after a failed epoch.
        }
      }, rclcpp::ServicesQoS(), control_group_);
    status_ = create_service<std_srvs::srv::Trigger>(std::string(get_name()) + "/is_active",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r) {
        std::lock_guard<std::mutex> lock(control_mutex_);
        r->success = active_; r->message = failed_ ? "FAILED_RESTART_REQUIRED" : "";
      }, rclcpp::ServicesQoS(), control_group_);
    // Bonds run in a separate callback group while finite lifecycle calls execute.
    tick_ = create_wall_timer(200ms, [this]() {
      std::lock_guard<std::mutex> lock(control_mutex_);
      if (closing_) {return;}
      if (autostart_) {autostart_ = false; startup();}
      if (active_) {
        for (const auto & entry : bonds_) {
          if (entry.second->isBroken()) {
            RCLCPP_ERROR(get_logger(), "BOND_BROKEN %s", entry.first.c_str());
            failed_ = true; stop_and_deactivate(); break;
          }
        }
      }
    }, control_group_);
    // Bond destruction creates its disconnect timer. It must happen before
    // rcl_shutdown invalidates the context, including on asynchronous SIGINT.
    pre_shutdown_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback([this]() {
      closing_ = true;
      tick_->cancel();
      rpc_executor_.cancel();
      std::lock_guard<std::mutex> lock(control_mutex_);
      active_ = false;
      bonds_.clear();
    });
  }

  ~BoundedManager() override
  {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(pre_shutdown_);
  }

private:
  template<class S>
  typename S::Response::SharedPtr call(
    typename rclcpp::Client<S>::SharedPtr client, typename S::Request::SharedPtr request)
  {
    if (closing_ || !client->wait_for_service(2s) || closing_) {return nullptr;}
    auto future = client->async_send_request(request);
    if (rpc_executor_.spin_until_future_complete(future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
      client->remove_pending_request(future); return nullptr;
    }
    return future.get();
  }

  bool transition(const std::string & name, uint8_t from, uint8_t to, uint8_t id)
  {
    // A timed-out mutation may have succeeded. Read state before any replay.
    for (int attempt = 0; attempt < 3 && rclcpp::ok() && !closing_; ++attempt) {
      auto state = call<Get>(get_[name], std::make_shared<Get::Request>());
      if (!state) {RCLCPP_WARN(get_logger(), "STATE_RPC_TIMEOUT %s", name.c_str()); continue;}
      if (state->current_state.id == to) {return true;}
      if (state->current_state.id != from) {
        RCLCPP_ERROR(get_logger(), "UNEXPECTED_STATE %s %u", name.c_str(), state->current_state.id);
        return false;
      }
      auto request = std::make_shared<Change::Request>(); request->transition.id = id;
      auto result = call<Change>(change_[name], request);
      if (result && !result->success) {return false;}
      if (!result) {RCLCPP_WARN(get_logger(), "TRANSITION_ACK_TIMEOUT %s; reconciling", name.c_str());}
      auto check = call<Get>(get_[name], std::make_shared<Get::Request>());
      if (check && check->current_state.id == to) {return true;}
    }
    RCLCPP_ERROR(get_logger(), "LIFECYCLE_FAILED %s transition=%u", name.c_str(), id);
    return false;
  }

  bool startup()
  {
    for (auto & name : names_) {
      if (!transition(name, 1, 2, 1)) {failed_ = true; stop_and_deactivate(); return false;}
    }
    for (auto & name : names_) {
      if (!transition(name, 2, 3, 3)) {failed_ = true; stop_and_deactivate(); return false;}
      if (closing_) {return false;}
      auto bond = std::make_shared<bond::Bond>("bond", name, shared_from_this());
      bond->setHeartbeatTimeout(4.0); bond->setHeartbeatPeriod(.10); bond->start();
      bonds_[name] = bond;
      if (!bond->waitUntilFormed(rclcpp::Duration::from_seconds(2.0))) {
        RCLCPP_ERROR(get_logger(), "BOND_NOT_FORMED %s", name.c_str());
        failed_ = true; stop_and_deactivate(); return false;
      }
    }
    if (closing_) {return false;}
    active_ = true;
    RCLCPP_INFO(get_logger(), "Managed nodes are active (bounded state-confirmed startup)");
    return true;
  }

  void stop_and_deactivate()
  {
    active_ = false;
    if (!closing_ && stop_->service_is_ready()) {
      stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }
    for (auto it = names_.rbegin(); it != names_.rend(); ++it) {
      auto state = call<Get>(get_[*it], std::make_shared<Get::Request>());
      if (state && state->current_state.id == 3) {transition(*it, 3, 2, 4);}
    }
    bonds_.clear();
  }

  bool autostart_{false}, active_{false}, failed_{false};
  std::atomic<bool> closing_{false};
  std::mutex control_mutex_;
  rclcpp::PreShutdownCallbackHandle pre_shutdown_;
  std::vector<std::string> names_;
  rclcpp::CallbackGroup::SharedPtr rpc_group_, control_group_;
  rclcpp::executors::SingleThreadedExecutor rpc_executor_;
  std::map<std::string, rclcpp::Client<Get>::SharedPtr> get_;
  std::map<std::string, rclcpp::Client<Change>::SharedPtr> change_;
  std::map<std::string, std::shared_ptr<bond::Bond>> bonds_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_;
  rclcpp::Service<Manage>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr tick_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BoundedManager>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
