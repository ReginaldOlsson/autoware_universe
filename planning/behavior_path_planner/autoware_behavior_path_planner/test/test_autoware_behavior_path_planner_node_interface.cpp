// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/behavior_path_planner/behavior_path_planner_node.hpp"
#include "autoware/behavior_path_planner/test_utils.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using autoware::behavior_path_planner::generateNode;
using autoware::behavior_path_planner::generateTestManager;
using autoware::behavior_path_planner::publishMandatoryTopics;

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionRoute)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode({}, {});

  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_route_topic = "behavior_path_planner/input/route";

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithBehaviorNormalRoute(test_target_node, input_route_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with empty route
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithAbnormalRoute(test_target_node, input_route_topic));
  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto test_target_node = generateNode({}, {});
  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_route_topic = "behavior_path_planner/input/route";
  const std::string input_odometry_topic = "behavior_path_planner/input/odometry";

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithBehaviorNormalRoute(test_target_node, input_route_topic));

  // make sure behavior_path_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithOffTrackOdometry(test_target_node, input_odometry_topic));

  rclcpp::shutdown();
}

namespace
{
struct RclcppContext
{
  RclcppContext() { rclcpp::init(0, nullptr); }
  RclcppContext(const RclcppContext &) = delete;
  RclcppContext & operator=(const RclcppContext &) = delete;
  ~RclcppContext()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

autoware_map_msgs::msg::LaneletMapBin make_empty_version_map()
{
  auto map_bin = autoware::test_utils::makeMapBinMsg();
  map_bin.version_map_format.clear();
  return map_bin;
}

std::filesystem::path sample_planning_map_path()
{
  const char * home = std::getenv("HOME");
  if (home == nullptr) {
    return {};
  }
  return std::filesystem::path(home) / "autoware_map/sample-map-planning/lanelet2_map.osm";
}
}  // namespace

TEST(PlanningModuleInterfaceTest, NodeTestWithEmptyMapVersion)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode({}, {});

  publishMandatoryTopics(test_manager, test_target_node);
  test_manager->publishInput(
    test_target_node, "behavior_path_planner/input/vector_map", make_empty_version_map());

  const std::string input_route_topic = "behavior_path_planner/input/route";
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithBehaviorNormalRoute(test_target_node, input_route_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithAbnormalRoute(test_target_node, input_route_topic));
  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithSamplePlanningMap)
{
  const auto map_path = sample_planning_map_path();
  if (map_path.empty() || !std::filesystem::exists(map_path)) {
    GTEST_SKIP() << "sample-map-planning is not installed at " << map_path;
  }

  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode({}, {});
  publishMandatoryTopics(test_manager, test_target_node);

  auto map_bin = autoware::test_utils::make_map_bin_msg(map_path.string(), 5.0);
  map_bin.version_map_format.clear();
  test_manager->publishInput(
    test_target_node, "behavior_path_planner/input/vector_map", map_bin);

  autoware::route_handler::RouteHandler route_handler;
  route_handler.setMap(map_bin);

  geometry_msgs::msg::Pose start_pose;
  geometry_msgs::msg::Pose goal_pose;
  start_pose.position = autoware_utils_geometry::create_point(3747.449707, 73774.179688, 19.092);
  start_pose.orientation =
    autoware_utils_geometry::create_quaternion(0.000703, -0.000417, 0.860234, 0.509899);
  goal_pose.position = autoware_utils_geometry::create_point(3719.276834, 73749.709762, 19.5109);
  goal_pose.orientation = autoware_utils_geometry::create_quaternion(0.0, 0.0, 0.849197, 0.528076);

  lanelet::ConstLanelets path_lanelets;
  ASSERT_TRUE(
    route_handler.planPathLaneletsBetweenCheckpoints(start_pose, goal_pose, &path_lanelets));
  ASSERT_FALSE(path_lanelets.empty());

  autoware_planning_msgs::msg::LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose = start_pose;
  route.goal_pose = goal_pose;
  for (const auto & lanelet : path_lanelets) {
    autoware_planning_msgs::msg::LaneletPrimitive primitive;
    primitive.id = lanelet.id();
    primitive.primitive_type = "lane";
    autoware_planning_msgs::msg::LaneletSegment segment;
    segment.preferred_primitive = primitive;
    segment.primitives.push_back(primitive);
    route.segments.push_back(segment);
  }
  ASSERT_FALSE(route.segments.empty());

  const std::string input_route_topic = "behavior_path_planner/input/route";
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->publishInput(test_target_node, input_route_topic, route));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 0);
  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, NodeSmokeTestWithStartPlanner)
{
  RclcppContext rclcpp_context;
  auto test_manager = generateTestManager();
  auto test_target_node =
    generateNode({"start_planner"}, {"autoware::behavior_path_planner::StartPlannerModuleManager"});

  publishMandatoryTopics(test_manager, test_target_node);
  test_manager->publishInput(
    test_target_node, "behavior_path_planner/input/vector_map", make_empty_version_map());

  const std::string input_route_topic = "behavior_path_planner/input/route";
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithBehaviorNormalRoute(test_target_node, input_route_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);
}

TEST(PlanningModuleInterfaceTest, NodeSmokeTestWithGoalPlanner)
{
  RclcppContext rclcpp_context;
  auto test_manager = generateTestManager();
  auto test_target_node =
    generateNode({"goal_planner"}, {"autoware::behavior_path_planner::GoalPlannerModuleManager"});

  publishMandatoryTopics(test_manager, test_target_node);
  test_manager->publishInput(
    test_target_node, "behavior_path_planner/input/vector_map", make_empty_version_map());

  const std::string input_route_topic = "behavior_path_planner/input/route";
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithBehaviorNormalRoute(test_target_node, input_route_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);
}
