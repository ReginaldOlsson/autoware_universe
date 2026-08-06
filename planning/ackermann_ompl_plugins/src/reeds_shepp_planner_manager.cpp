#include "ackermann_ompl_plugins/reeds_shepp_planner_manager.hpp"
#include "ackermann_ompl_plugins/reeds_shepp_planning_context.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace ackermann_ompl_plugins
{

bool ReedsSheppPlannerManager::initialize(
  const moveit::core::RobotModelConstPtr & model, const rclcpp::Node::SharedPtr & node,
  const std::string & parameter_namespace)
{
  node_ = node;
  robot_model_ = model;

  const std::string key =
    parameter_namespace.empty() ? "turning_radius" : parameter_namespace + ".turning_radius";
  if (!node_->has_parameter(key)) {
    node_->declare_parameter<double>(key, turning_radius_);
  }
  node_->get_parameter(key, turning_radius_);

  RCLCPP_INFO(
    node_->get_logger(), "ReedsSheppPlannerManager initialized with turning_radius=%.2f",
    turning_radius_);
  return true;
}

void ReedsSheppPlannerManager::getPlanningAlgorithms(std::vector<std::string> & algs) const
{
  algs = {"RRTstar", "RRTConnect", "PRM", "ReedsShepp"};
}

planning_interface::PlanningContextPtr ReedsSheppPlannerManager::getPlanningContext(
  const planning_scene::PlanningSceneConstPtr & planning_scene,
  const planning_interface::MotionPlanRequest & req,
  moveit_msgs::msg::MoveItErrorCodes & error_code) const
{
  auto context = std::make_shared<ReedsSheppPlanningContext>(
    "ReedsShepp", req.group_name, robot_model_, turning_radius_);
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(req);
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return context;
}

void ReedsSheppPlannerManager::setPlannerConfigurations(
  const planning_interface::PlannerConfigurationMap & /*pcs*/)
{
}

}  // namespace ackermann_ompl_plugins

PLUGINLIB_EXPORT_CLASS(
  ackermann_ompl_plugins::ReedsSheppPlannerManager, planning_interface::PlannerManager)
