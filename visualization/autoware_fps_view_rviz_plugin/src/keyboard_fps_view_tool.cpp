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

#include "keyboard_fps_view_tool.hpp"

#include "keyboard_fps_view_controller.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>

#include <QCoreApplication>
#include <QEvent>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QString>
#include <QWidget>

namespace autoware::fps_view_rviz_plugin
{
namespace
{
bool is_movement_key(const int key)
{
  return key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D ||
         key == Qt::Key_Q || key == Qt::Key_E || key == Qt::Key_Shift;
}
}  // namespace

KeyboardFpsViewTool::KeyboardFpsViewTool()
{
  shortcut_key_ = 'v';
  access_all_keys_ = true;
}

KeyboardFpsViewTool::~KeyboardFpsViewTool()
{
  remove_key_filter();
}

void KeyboardFpsViewTool::onInitialize()
{
  setName("Keyboard FPS");

  movement_speed_property_ = new rviz_common::properties::FloatProperty(
    "Movement Speed", 1.0, "Camera movement speed in meters per second while a key is held.",
    getPropertyContainer(), SLOT(on_movement_speed_changed()), this);
  movement_speed_property_->setMin(0.0);
  movement_speed_property_->setReadOnly(false);

  boost_factor_property_ = new rviz_common::properties::FloatProperty(
    "Boost Factor", 1.0,
    "Extra speed multiplier applied while Shift is held. Final speed is Movement Speed * (1 + "
    "Boost Factor).",
    getPropertyContainer(), SLOT(on_boost_factor_changed()), this);
  boost_factor_property_->setMin(0.0);
  boost_factor_property_->setReadOnly(false);

  fly_mode_property_ = new rviz_common::properties::BoolProperty(
    "Fly Mode", true,
    "If enabled, movement follows the look direction. If disabled, WASD stays on the XY plane and "
    "Q/E change world height.",
    getPropertyContainer(), SLOT(on_fly_mode_changed()), this);
  fly_mode_property_->setReadOnly(false);
}

void KeyboardFpsViewTool::activate()
{
  rviz_common::ViewManager * view_manager = context_->getViewManager();
  rviz_common::ViewController * current_view = view_manager->getCurrent();
  const QString view_id = QStringLiteral("autoware_fps_view_rviz_plugin/KeyboardFPS");
  if (current_view == nullptr || current_view->getClassId() != view_id) {
    view_manager->setCurrentViewControllerType(view_id);
  }

  apply_properties_to_view();
  KeyboardFpsViewController * fps_view = current_fps_view();
  if (fps_view != nullptr) {
    fps_view->stop_keyboard_motion();
    fps_view->set_keyboard_input_enabled(true);
  }
  install_key_filter();
}

void KeyboardFpsViewTool::deactivate()
{
  remove_key_filter();
  KeyboardFpsViewController * fps_view = current_fps_view();
  if (fps_view != nullptr) {
    fps_view->set_keyboard_input_enabled(false);
  }
}

KeyboardFpsViewController * KeyboardFpsViewTool::current_fps_view() const
{
  if (context_ == nullptr || context_->getViewManager() == nullptr) {
    return nullptr;
  }
  return dynamic_cast<KeyboardFpsViewController *>(context_->getViewManager()->getCurrent());
}

void KeyboardFpsViewTool::apply_properties_to_view()
{
  KeyboardFpsViewController * fps_view = current_fps_view();
  if (fps_view == nullptr) {
    return;
  }
  fps_view->set_movement_speed(movement_speed_property_->getFloat());
  fps_view->set_boost_factor(boost_factor_property_->getFloat());
  fps_view->set_fly_mode(fly_mode_property_->getBool());
}

void KeyboardFpsViewTool::stop_view_motion()
{
  KeyboardFpsViewController * fps_view = current_fps_view();
  if (fps_view != nullptr) {
    fps_view->stop_keyboard_motion();
  }
}

void KeyboardFpsViewTool::on_movement_speed_changed()
{
  KeyboardFpsViewController * fps_view = current_fps_view();
  if (fps_view != nullptr) {
    fps_view->set_movement_speed(movement_speed_property_->getFloat());
  }
}

void KeyboardFpsViewTool::on_boost_factor_changed()
{
  KeyboardFpsViewController * fps_view = current_fps_view();
  if (fps_view != nullptr) {
    fps_view->set_boost_factor(boost_factor_property_->getFloat());
  }
}

void KeyboardFpsViewTool::on_fly_mode_changed()
{
  KeyboardFpsViewController * fps_view = current_fps_view();
  if (fps_view != nullptr) {
    fps_view->set_fly_mode(fly_mode_property_->getBool());
  }
}

void KeyboardFpsViewTool::install_key_filter()
{
  if (key_filter_installed_) {
    return;
  }
  if (QCoreApplication::instance() != nullptr) {
    QCoreApplication::instance()->installEventFilter(this);
    key_filter_installed_ = true;
  }
}

void KeyboardFpsViewTool::remove_key_filter()
{
  if (QWidget * grabber = QWidget::keyboardGrabber()) {
    grabber->releaseKeyboard();
  }
  if (!key_filter_installed_) {
    return;
  }
  if (QCoreApplication::instance() != nullptr) {
    QCoreApplication::instance()->removeEventFilter(this);
  }
  key_filter_installed_ = false;
}

void KeyboardFpsViewTool::forward_key_event(QKeyEvent * event)
{
  rviz_common::RenderPanel * panel = context_->getViewManager()->getRenderPanel();
  if (panel != nullptr && panel->getViewController() != nullptr) {
    panel->getViewController()->handleKeyEvent(event, panel);
  }
}

bool KeyboardFpsViewTool::eventFilter(QObject * object, QEvent * event)
{
  (void)object;

  // Only stop when RViz itself loses OS focus. Mouse-look uses a native render
  // window, which can fire WindowDeactivate without the user leaving the view.
  if (event->type() == QEvent::ApplicationDeactivate) {
    stop_view_motion();
    return false;
  }
  if (event->type() == QEvent::ApplicationStateChange) {
    if (QGuiApplication::applicationState() != Qt::ApplicationActive) {
      stop_view_motion();
    }
    return false;
  }

  if (event->type() == QEvent::ShortcutOverride) {
    auto * key_event = static_cast<QKeyEvent *>(event);
    if (is_movement_key(key_event->key())) {
      key_event->accept();
      return true;
    }
    return false;
  }

  if (event->type() != QEvent::KeyPress && event->type() != QEvent::KeyRelease) {
    return false;
  }

  auto * key_event = static_cast<QKeyEvent *>(event);
  if (!is_movement_key(key_event->key())) {
    return false;
  }

  // Keep held keys while the mouse is used to look around. Auto-repeat and
  // focus changes from the render window must not clear WASD state.
  if (event->type() == QEvent::KeyRelease && !key_event->isAutoRepeat()) {
    forward_key_event(key_event);
    return false;
  }

  if (event->type() == QEvent::KeyPress && !key_event->isAutoRepeat()) {
    forward_key_event(key_event);
  }
  return false;
}

int KeyboardFpsViewTool::processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel)
{
  if (panel != nullptr && panel->getViewController() != nullptr) {
    panel->getViewController()->handleKeyEvent(event, panel);
  }
  return Render;
}

int KeyboardFpsViewTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.panel != nullptr && event.panel->getViewController() != nullptr) {
    event.panel->getViewController()->handleMouseEvent(event);
    setCursor(event.panel->getViewController()->getCursor());
  }
  return 0;
}

}  // namespace autoware::fps_view_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware::fps_view_rviz_plugin::KeyboardFpsViewTool, rviz_common::Tool)
