// Jazzy rclpy omits publisher_gid in MessageInfo. Observe actual RMW identity here.
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <set>
#include <sstream>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("w09_tf_authority");
  std::set<std::string> seen;
  auto sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(1000),
    [&](tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo &info) {
      const auto &gid = info.get_rmw_message_info().publisher_gid;
      std::ostringstream hex;
      for (auto v : gid.data) hex << std::hex << std::setw(2) << std::setfill('0') << int(v);
      std::string publisher = "unresolved";
      for (const auto &endpoint : node->get_publishers_info_by_topic("/tf")) {
        if (std::memcmp(endpoint.endpoint_gid().data(), gid.data, RMW_GID_STORAGE_SIZE) == 0)
          publisher = endpoint.node_name();
      }
      for (const auto &t : msg->transforms) {
        auto key = hex.str() + t.child_frame_id + publisher;
        if (seen.insert(key).second) {
          std::printf("{\"parent\":\"%s\",\"child\":\"%s\",\"gid\":\"%s\",\"node\":\"%s\"}\n",
                      t.header.frame_id.c_str(), t.child_frame_id.c_str(), hex.str().c_str(), publisher.c_str());
          std::fflush(stdout);
        }
      }
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
