#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace ackermann_ompl_plugins
{

class ReedsSheppPlannerManager : public planning_interface::PlannerManager
{
public:
  bool initialize(
    const moveit::core::RobotModelConstPtr & model, const rclcpp::Node::SharedPtr & node,
    const std::string & parameter_namespace) override;

  std::string getDescription() const override { return "ReedsShepp"; }

  void getPlanningAlgorithms(std::vector<std::string> & algs) const override;

  planning_interface::PlanningContextPtr getPlanningContext(
    const planning_scene::PlanningSceneConstPtr & planning_scene,
    const planning_interface::MotionPlanRequest & req,
    moveit_msgs::msg::MoveItErrorCodes & error_code) const override;

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap & pcs) override;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Node::SharedPtr node_;
  double turning_radius_{6.0};
};

}  // namespace ackermann_ompl_plugins
