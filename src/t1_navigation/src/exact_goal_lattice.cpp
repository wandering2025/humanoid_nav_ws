// Preserve the requested goal yaw after the installed forward lattice search.
// Lattice search discretizes yaw (0 / 0.464 rad around zero). Those search bins
// must not silently replace the continuous target of a terminal correction.
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace t1_navigation
{
class ExactGoalLattice : public nav2_smac_planner::SmacPlannerLattice
{
public:
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override
  {
    auto path = SmacPlannerLattice::createPlan(start, goal, cancel_checker);
    // W11 uses a circular 2.05 m planning footprint: rotating the endpoint
    // does not change collision clearance. Keep every translation sample.
    // The existing controller performs the allowed in-place heading change
    // through the unchanged adapter -> W10 gate -> smoother -> CM -> bridge.
    if (!path.poses.empty()) {
      path.poses.back().pose.orientation = goal.pose.orientation;
    }
    return path;
  }
};
}  // namespace t1_navigation

PLUGINLIB_EXPORT_CLASS(t1_navigation::ExactGoalLattice, nav2_core::GlobalPlanner)
