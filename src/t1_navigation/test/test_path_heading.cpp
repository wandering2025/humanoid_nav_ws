// Score actual installed MPPI critic outputs, not a duplicate cost formula.
#include "gtest/gtest.h"
#include "nav2_mppi_controller/critics/path_align_critic.hpp"
#include "yaml-cpp/yaml.h"

class ConfiguredAlignment : public mppi::critics::PathAlignCritic
{
public:
  explicit ConfiguredAlignment(bool orientations)
  {
    const auto p = YAML::LoadFile(T1_NAVIGATION_CONFIG)["controller_server"]
      ["ros__parameters"]["FollowPath"]["PathAlignCritic"];
    enabled_ = p["enabled"].as<bool>();
    power_ = p["cost_power"].as<unsigned>();
    weight_ = p["cost_weight"].as<float>();
    threshold_to_consider_ = p["threshold_to_consider"].as<float>();
    max_path_occupancy_ratio_ = p["max_path_occupancy_ratio"].as<float>();
    offset_from_furthest_ = p["offset_from_furthest"].as<size_t>();
    trajectory_point_step_ = p["trajectory_point_step"].as<int>();
    use_path_orientations_ = orientations;
  }
};

TEST(TerminalHeading, SamePositionDifferentHeadingMustHaveDifferentCost)
{
  const auto p = YAML::LoadFile(T1_NAVIGATION_CONFIG)["controller_server"]
    ["ros__parameters"]["FollowPath"]["PathAlignCritic"];
  mppi::models::State state;
  state.pose.pose.orientation.w = 1.;
  mppi::models::Trajectories trajectories;
  trajectories.reset(2, 52);
  mppi::models::Path path;
  path.reset(60);
  for (size_t i = 0; i < 60; ++i) {path.x(i) = .05f * i;}
  for (size_t i = 0; i < 52; ++i) {
    trajectories.x(0, i) = trajectories.x(1, i) = .025f * i;
    trajectories.yaws(1, i) = 1.5708f;
  }
  geometry_msgs::msg::Pose goal;
  goal.position.x = 4.; goal.orientation.w = 1.;
  auto costs = xt::xtensor<float, 1>(xt::zeros<float>({2}));
  float dt = .05;
  mppi::CriticData data{state, trajectories, path, goal, costs, dt, false,
    nullptr, nullptr, std::vector<bool>(60, true), size_t(40)};
  ConfiguredAlignment configured(p["use_path_orientations"].as<bool>());
  configured.score(data);
  EXPECT_GT(costs(1), costs(0) + 1.0f);
  // Negative control: the old XY-only setting cannot distinguish these inputs.
  costs.fill(0.);
  ConfiguredAlignment old(false);
  old.score(data);
  EXPECT_FLOAT_EQ(costs(0), costs(1));
}
