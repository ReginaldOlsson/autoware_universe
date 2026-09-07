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

#include "parking_spot_panel.hpp"

#include <QHBoxLayout>
#include <QListWidgetItem>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <string>

namespace autoware::parking_spot_rviz_plugin
{
ParkingSpotPanel::ParkingSpotPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  search_edit_ = new QLineEdit;
  search_edit_->setPlaceholderText("Search parking spots");

  spot_list_ = new QListWidget;

  set_goal_button_ = new QPushButton("Set Goal");
  set_goal_button_->setEnabled(false);

  goal_topic_edit_ = new QLineEdit("/planning/mission_planning/goal");

  status_label_ = new QLabel("Waiting for /map/vector_map");

  auto * topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Goal topic:"));
  topic_layout->addWidget(goal_topic_edit_);

  auto * layout = new QVBoxLayout;
  layout->addWidget(search_edit_);
  layout->addWidget(spot_list_);
  layout->addWidget(set_goal_button_);
  layout->addLayout(topic_layout);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(search_edit_, &QLineEdit::textChanged, this, &ParkingSpotPanel::onSearchTextChanged);
  connect(set_goal_button_, &QPushButton::clicked, this, &ParkingSpotPanel::onSetGoalClicked);
  connect(
    spot_list_, &QListWidget::itemDoubleClicked, this, &ParkingSpotPanel::onItemDoubleClicked);
  connect(
    goal_topic_edit_, &QLineEdit::editingFinished, this,
    &ParkingSpotPanel::onGoalTopicEditingFinished);
}

void ParkingSpotPanel::onInitialize()
{
  raw_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  sub_map_ = raw_node_->create_subscription<LaneletMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ParkingSpotPanel::onMap, this, std::placeholders::_1));
  updateGoalPublisher();
}

void ParkingSpotPanel::onMap(const LaneletMapBin::ConstSharedPtr msg)
{
  route_handler_.setMap(*msg);
  spots_ = route_handler_.getNamedParkingSpots();
  map_frame_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
  refreshSpotList();

  if (spots_.empty()) {
    status_label_->setText("No named parking spots in the map");
  } else {
    status_label_->setText(QString("%1 named parking spots").arg(spots_.size()));
  }
}

void ParkingSpotPanel::updateGoalPublisher()
{
  if (!raw_node_) {
    return;
  }

  const auto topic = goal_topic_edit_->text().toStdString();
  if (topic.empty() || topic == current_goal_topic_) {
    return;
  }

  pub_goal_ = raw_node_->create_publisher<PoseStamped>(topic, rclcpp::QoS{1});
  current_goal_topic_ = topic;
}

void ParkingSpotPanel::refreshSpotList()
{
  const auto filter = search_edit_->text().trimmed();
  spot_list_->clear();

  for (const auto & spot : spots_) {
    const auto name = QString::fromStdString(spot.name);
    if (!filter.isEmpty() && !name.contains(filter, Qt::CaseInsensitive)) {
      continue;
    }
    auto * item = new QListWidgetItem(name);
    item->setData(Qt::UserRole, name);
    spot_list_->addItem(item);
  }

  set_goal_button_->setEnabled(spot_list_->count() > 0);
  if (spot_list_->count() > 0) {
    spot_list_->setCurrentRow(0);
  }
}

void ParkingSpotPanel::publishSelectedSpot()
{
  if (!pub_goal_) {
    updateGoalPublisher();
  }
  if (!pub_goal_) {
    status_label_->setText("Goal topic is empty");
    return;
  }

  const auto * item = spot_list_->currentItem();
  if (!item) {
    status_label_->setText("Select a parking spot first");
    return;
  }

  const auto name = item->data(Qt::UserRole).toString().toStdString();
  const auto spot = route_handler_.getParkingSpotByName(name);
  if (!spot) {
    status_label_->setText(
      QString("Parking spot '%1' was not found").arg(QString::fromStdString(name)));
    return;
  }

  PoseStamped msg;
  msg.header.stamp = raw_node_->now();
  msg.header.frame_id = map_frame_;
  msg.pose = spot->pose;
  pub_goal_->publish(msg);

  status_label_->setText(
    QString("Published goal for '%1'").arg(QString::fromStdString(spot->name)));
}

void ParkingSpotPanel::onSearchTextChanged(const QString &)
{
  refreshSpotList();
}

void ParkingSpotPanel::onSetGoalClicked()
{
  publishSelectedSpot();
}

void ParkingSpotPanel::onItemDoubleClicked(QListWidgetItem *)
{
  publishSelectedSpot();
}

void ParkingSpotPanel::onGoalTopicEditingFinished()
{
  current_goal_topic_.clear();
  updateGoalPublisher();
}

}  // namespace autoware::parking_spot_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware::parking_spot_rviz_plugin::ParkingSpotPanel, rviz_common::Panel)
