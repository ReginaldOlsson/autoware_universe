// Copyright 2026 Autoware Foundation
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

#ifndef PARKING_SPOT_PANEL_HPP_
#define PARKING_SPOT_PANEL_HPP_

#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <autoware/route_handler/route_handler.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>
#include <vector>

namespace autoware::parking_spot_rviz_plugin
{
class ParkingSpotPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ParkingSpotPanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected Q_SLOTS:
  void onSearchTextChanged(const QString & text);
  void onSetGoalClicked();
  void onItemDoubleClicked(QListWidgetItem * item);
  void onGoalTopicEditingFinished();

private:
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using NamedParkingSpot = autoware::route_handler::NamedParkingSpot;

  void onMap(const LaneletMapBin::ConstSharedPtr msg);
  void updateGoalPublisher();
  void refreshSpotList();
  void publishSelectedSpot();

  QLineEdit * search_edit_{nullptr};
  QLineEdit * goal_topic_edit_{nullptr};
  QListWidget * spot_list_{nullptr};
  QPushButton * set_goal_button_{nullptr};
  QLabel * status_label_{nullptr};

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_goal_;
  autoware::route_handler::RouteHandler route_handler_;
  std::vector<NamedParkingSpot> spots_;
  std::string map_frame_{"map"};
  std::string current_goal_topic_;
};

}  // namespace autoware::parking_spot_rviz_plugin

#endif  // PARKING_SPOT_PANEL_HPP_
