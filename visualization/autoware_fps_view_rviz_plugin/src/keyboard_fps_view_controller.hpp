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

#ifndef KEYBOARD_FPS_VIEW_CONTROLLER_HPP_
#define KEYBOARD_FPS_VIEW_CONTROLLER_HPP_

#include <rviz_default_plugins/view_controllers/fps/fps_view_controller.hpp>

#include <OgreVector.h>

class QKeyEvent;

namespace rviz_common
{
class RenderPanel;
class ViewportMouseEvent;

namespace properties
{
class BoolProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace autoware::fps_view_rviz_plugin
{
class KeyboardFpsViewController : public rviz_default_plugins::view_controllers::FPSViewController
{
public:
  KeyboardFpsViewController();
  ~KeyboardFpsViewController() override;

  void onActivate() override;
  void handleMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  void handleKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;
  void update(float dt, float ros_dt) override;
  void stop_keyboard_motion();
  void set_keyboard_input_enabled(bool enabled);

  void set_movement_speed(float speed);
  void set_boost_factor(float boost_factor);
  void set_fly_mode(bool enabled);
  float movement_speed() const;
  float boost_factor() const;
  bool fly_mode() const;

private:
  void update_key_state(const QKeyEvent & event);
  Ogre::Vector3 held_local_direction() const;
  void apply_keyboard_motion(float dt);
  void make_keyboard_tool_current();
  void restore_default_tool();

  rviz_common::properties::FloatProperty * movement_speed_property_{nullptr};
  rviz_common::properties::FloatProperty * boost_factor_property_{nullptr};
  rviz_common::properties::BoolProperty * fly_mode_property_{nullptr};

  bool key_w_{false};
  bool key_a_{false};
  bool key_s_{false};
  bool key_d_{false};
  bool key_q_{false};
  bool key_e_{false};
  bool shift_pressed_{false};
  bool keyboard_input_enabled_{false};
};

}  // namespace autoware::fps_view_rviz_plugin

#endif  // KEYBOARD_FPS_VIEW_CONTROLLER_HPP_
