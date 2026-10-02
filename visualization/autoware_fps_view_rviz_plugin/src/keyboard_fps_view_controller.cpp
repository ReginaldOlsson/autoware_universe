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

#include "keyboard_fps_view_controller.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

#include <OgreVector.h>

#include <QKeyEvent>
#include <QString>

#include <algorithm>
#include <cmath>

namespace autoware::fps_view_rviz_plugin
{
namespace
{
const QString keyboard_fps_tool_id =
  QStringLiteral("autoware_fps_view_rviz_plugin/KeyboardFPSTool");
constexpr float max_frame_dt = 0.05f;
}  // namespace

KeyboardFpsViewController::KeyboardFpsViewController()
{
  movement_speed_property_ = new rviz_common::properties::FloatProperty(
    "Movement Speed", 5.0, "Camera movement speed in meters per second while a key is held.", this);
  movement_speed_property_->setMin(0.0);

  boost_factor_property_ = new rviz_common::properties::FloatProperty(
    "Boost Factor", 3.0,
    "Extra speed multiplier applied while Shift is held. Final speed is Movement Speed * (1 + "
    "Boost Factor).",
    this);
  boost_factor_property_->setMin(0.0);

  fly_mode_property_ = new rviz_common::properties::BoolProperty(
    "Fly Mode", true,
    "If enabled, movement follows the look direction. If disabled, WASD stays on the XY plane and "
    "Q/E change world height.",
    this);

  movement_speed_property_->setReadOnly(false);
  boost_factor_property_->setReadOnly(false);
  fly_mode_property_->setReadOnly(false);
}

void KeyboardFpsViewController::set_movement_speed(const float speed)
{
  movement_speed_property_->setFloat(speed);
}

void KeyboardFpsViewController::set_boost_factor(const float boost_factor)
{
  boost_factor_property_->setFloat(boost_factor);
}

void KeyboardFpsViewController::set_fly_mode(const bool enabled)
{
  fly_mode_property_->setBool(enabled);
}

float KeyboardFpsViewController::movement_speed() const
{
  return movement_speed_property_->getFloat();
}

float KeyboardFpsViewController::boost_factor() const
{
  return boost_factor_property_->getFloat();
}

bool KeyboardFpsViewController::fly_mode() const
{
  return fly_mode_property_->getBool();
}

KeyboardFpsViewController::~KeyboardFpsViewController()
{
  stop_keyboard_motion();
  restore_default_tool();
}

void KeyboardFpsViewController::onActivate()
{
  FPSViewController::onActivate();
  make_keyboard_tool_current();
}

void KeyboardFpsViewController::make_keyboard_tool_current()
{
  if (context_ == nullptr) {
    return;
  }

  rviz_common::ToolManager * tool_manager = context_->getToolManager();
  if (tool_manager == nullptr) {
    return;
  }

  rviz_common::Tool * fps_tool = nullptr;
  for (int i = 0; i < tool_manager->numTools(); ++i) {
    rviz_common::Tool * tool = tool_manager->getTool(i);
    if (tool != nullptr && tool->getClassId() == keyboard_fps_tool_id) {
      fps_tool = tool;
      break;
    }
  }

  if (fps_tool == nullptr) {
    fps_tool = tool_manager->addTool(keyboard_fps_tool_id);
  }

  if (fps_tool != nullptr && tool_manager->getCurrentTool() != fps_tool) {
    tool_manager->setCurrentTool(fps_tool);
  }
}

void KeyboardFpsViewController::restore_default_tool()
{
  if (context_ == nullptr) {
    return;
  }

  rviz_common::ToolManager * tool_manager = context_->getToolManager();
  if (tool_manager == nullptr) {
    return;
  }

  rviz_common::Tool * current = tool_manager->getCurrentTool();
  rviz_common::Tool * default_tool = tool_manager->getDefaultTool();
  if (
    current != nullptr && default_tool != nullptr && current != default_tool &&
    current->getClassId() == keyboard_fps_tool_id)
  {
    tool_manager->setCurrentTool(default_tool);
  }
}

void KeyboardFpsViewController::handleMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  FPSViewController::handleMouseEvent(event);
  setStatus(
    "<b>Left-Click:</b> Rotate.  <b>Hold W/A/S/D:</b> Move.  <b>Hold Q/E:</b> Down/Up.  "
    "<b>Shift:</b> Boost.");
}

void KeyboardFpsViewController::handleKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel)
{
  rviz_common::ViewController::handleKeyEvent(event, panel);
  if (!keyboard_input_enabled_ || event == nullptr || event->isAutoRepeat()) {
    return;
  }
  update_key_state(*event);
}

void KeyboardFpsViewController::update(const float dt, const float ros_dt)
{
  FPSViewController::update(dt, ros_dt);
  apply_keyboard_motion(dt);
}

void KeyboardFpsViewController::stop_keyboard_motion()
{
  key_w_ = false;
  key_a_ = false;
  key_s_ = false;
  key_d_ = false;
  key_q_ = false;
  key_e_ = false;
  shift_pressed_ = false;
}

void KeyboardFpsViewController::set_keyboard_input_enabled(const bool enabled)
{
  keyboard_input_enabled_ = enabled;
  if (!enabled) {
    stop_keyboard_motion();
  }
}

void KeyboardFpsViewController::update_key_state(const QKeyEvent & event)
{
  const bool pressed = event.type() == QEvent::KeyPress;

  switch (event.key()) {
    case Qt::Key_W:
      key_w_ = pressed;
      break;
    case Qt::Key_A:
      key_a_ = pressed;
      break;
    case Qt::Key_S:
      key_s_ = pressed;
      break;
    case Qt::Key_D:
      key_d_ = pressed;
      break;
    case Qt::Key_Q:
      key_q_ = pressed;
      break;
    case Qt::Key_E:
      key_e_ = pressed;
      break;
    case Qt::Key_Shift:
      shift_pressed_ = pressed;
      break;
    default:
      shift_pressed_ = (event.modifiers() & Qt::ShiftModifier) != 0;
      break;
  }
}

Ogre::Vector3 KeyboardFpsViewController::held_local_direction() const
{
  Ogre::Vector3 direction(0.0f, 0.0f, 0.0f);
  if (key_d_) {
    direction.x += 1.0f;
  }
  if (key_a_) {
    direction.x -= 1.0f;
  }
  if (key_e_) {
    direction.y += 1.0f;
  }
  if (key_q_) {
    direction.y -= 1.0f;
  }
  if (key_s_) {
    direction.z += 1.0f;
  }
  if (key_w_) {
    direction.z -= 1.0f;
  }
  if (direction.squaredLength() > 0.0f) {
    direction.normalise();
  }
  return direction;
}

void KeyboardFpsViewController::apply_keyboard_motion(const float dt)
{
  if (!keyboard_input_enabled_) {
    return;
  }

  const Ogre::Vector3 direction = held_local_direction();
  if (direction.squaredLength() == 0.0f) {
    return;
  }

  const float clamped_dt = std::min(std::max(dt, 0.0f), max_frame_dt);
  float speed = movement_speed_property_->getFloat();
  if (shift_pressed_) {
    speed *= (1.0f + boost_factor_property_->getFloat());
  }

  const Ogre::Vector3 delta = direction * speed * clamped_dt;
  if (fly_mode_property_->getBool()) {
    move(delta.x, delta.y, delta.z);  // NOLINT(build/include_what_you_use)
  } else {
    Ogre::Vector3 horizontal = getOrientation() * Ogre::Vector3(delta.x, 0.0f, delta.z);
    horizontal.z = 0.0f;
    position_property_->add(horizontal);
    if (std::fabs(delta.y) > 0.0f) {
      position_property_->add(Ogre::Vector3(0.0f, 0.0f, delta.y));
    }
  }

  if (context_ != nullptr) {
    context_->queueRender();
  }
}

}  // namespace autoware::fps_view_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::fps_view_rviz_plugin::KeyboardFpsViewController, rviz_common::ViewController)
