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

#ifndef KEYBOARD_FPS_VIEW_TOOL_HPP_
#define KEYBOARD_FPS_VIEW_TOOL_HPP_

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

#include <QObject>

class QEvent;

namespace autoware::fps_view_rviz_plugin
{
class KeyboardFpsViewController;

class KeyboardFpsViewTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  KeyboardFpsViewTool();
  ~KeyboardFpsViewTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  bool eventFilter(QObject * object, QEvent * event) override;

private Q_SLOTS:
  void on_movement_speed_changed();
  void on_boost_factor_changed();
  void on_fly_mode_changed();

private:
  void install_key_filter();
  void remove_key_filter();
  void forward_key_event(QKeyEvent * event);
  void apply_properties_to_view();
  void stop_view_motion();
  KeyboardFpsViewController * current_fps_view() const;

  rviz_common::properties::FloatProperty * movement_speed_property_{nullptr};
  rviz_common::properties::FloatProperty * boost_factor_property_{nullptr};
  rviz_common::properties::BoolProperty * fly_mode_property_{nullptr};
  bool key_filter_installed_{false};
};

}  // namespace autoware::fps_view_rviz_plugin

#endif  // KEYBOARD_FPS_VIEW_TOOL_HPP_
