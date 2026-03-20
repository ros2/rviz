// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2017, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is a copy of visualization_manager.hpp as it existed BEFORE
// commit 9716a8a ("Use a thread to run ROS executor"), preserved under a
// different class name for benchmark comparisons.
//
// Key difference from VisualizationManager (post-9716a8a):
//   - No std::thread executor_thread_ member.
//   - onUpdate() calls executor_->spin_some(10 ms) directly (old pattern).

#ifndef RVIZ_COMMON__VISUALIZATION_MANAGER_SPIN_SOME_HPP_
#define RVIZ_COMMON__VISUALIZATION_MANAGER_SPIN_SOME_HPP_

#include <deque>
#include <memory>

#include <QString>  // NOLINT: cpplint is unable to handle the include order here

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "rviz_common/bit_allocator.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/transformation/transformation_manager.hpp"

class QTimer;

namespace Ogre
{
class Light;
class Root;
}

namespace rviz_common
{

namespace properties
{

class ColorProperty;
class IntProperty;
class Property;
class PropertyTreeModel;
class StatusList;
class TfFrameProperty;

}  // namespace properties

class Display;
class Tool;
class OgreRenderQueueClearer;

class VisualizationManagerSpinSomePrivate;

/// Old VisualizationManager that drives the ROS executor from the update loop.
/**
 * Identical to VisualizationManager before commit 9716a8a.
 * The executor is driven by calling spin_some(10ms) inside onUpdate(), which
 * runs at ~30 Hz on the Qt main thread via a QTimer.
 *
 * This class is kept for benchmark purposes only.
 * Use VisualizationManager (with its dedicated executor thread) in production.
 */
class RVIZ_COMMON_PUBLIC VisualizationManagerSpinSome : public DisplayContext
{
  Q_OBJECT

public:
  explicit VisualizationManagerSpinSome(
    RenderPanel * render_panel,
    ros_integration::RosNodeAbstractionIface::WeakPtr ros_node_abstraction,
    WindowManagerInterface * wm, rclcpp::Clock::SharedPtr clock);

  ~VisualizationManagerSpinSome() override;

  void initialize();

  void startUpdate();

  void stopUpdate();

  Display * createDisplay(const QString & class_lookup_name, const QString & name, bool enabled);

  void addDisplay(Display * display, bool enabled);

  void removeAllDisplays();

  void load(const Config & config);

  void save(Config config) const;

  QString getFixedFrame() const override;

  void setFixedFrame(const QString & frame);

  Ogre::SceneManager * getSceneManager() const override;

  RenderPanel * getRenderPanel() const;

  double getWallClock();

  double getROSTime();

  double getWallClockElapsed();

  double getROSTimeElapsed();

  void handleChar(QKeyEvent * event, RenderPanel * panel) override;

  void handleMouseEvent(const ViewportMouseEvent & event) override;

  std::shared_ptr<rviz_common::interaction::HandlerManagerIface>
  getHandlerManager() const override;

  std::shared_ptr<rviz_common::interaction::SelectionManagerIface>
  getSelectionManager() const override;

  std::shared_ptr<rviz_common::interaction::ViewPickerIface> getViewPicker() const override;

  ToolManager * getToolManager() const override;

  ViewManager * getViewManager() const override;

  rviz_common::transformation::TransformationManager * getTransformationManager() override;

  void lockRender() override;

  void unlockRender() override;

  void queueRender() override;

  WindowManagerInterface * getWindowManager() const override;

  ros_integration::RosNodeAbstractionIface::WeakPtr getRosNodeAbstraction() const override;

  FrameManagerIface * getFrameManager() const override;

  uint64_t getFrameCount() const override;

  void notifyConfigChanged();

  DisplayFactory * getDisplayFactory() const override;

  properties::PropertyTreeModel * getDisplayTreeModel() const;

  void emitStatusUpdate(const QString & message);

  DisplayGroup * getRootDisplayGroup() const override;

  uint32_t getDefaultVisibilityBit() const override;

  BitAllocator * visibilityBits() override;

  void setStatus(const QString & message) override;

  virtual void setHelpPath(const QString & help_path);

  QString getHelpPath() const override;

  rclcpp::Clock::SharedPtr getClock() override;

public Q_SLOTS:
  void resetTime();

Q_SIGNALS:
  void timeJumped();
  void preUpdate();
  void configChanged();
  void statusUpdate(const QString & message);
  void escapePressed();

protected Q_SLOTS:
  /// Drives the ROS executor via spin_some(10ms) before processing displays.
  /// This is the OLD pattern replaced in commit 9716a8a.
  void onUpdate();

  void onToolChanged(Tool *);

protected:
  void onTimeJump(const rcl_time_jump_t & time_jump);

  void updateTime();

  void updateFrames();

  Ogre::Root * ogre_root_;
  QTimer * update_timer_;
  rclcpp::Time last_update_ros_time_;
  std::chrono::system_clock::time_point last_update_wall_time_;
  volatile bool shutting_down_;
  properties::PropertyTreeModel * display_property_tree_model_;
  DisplayGroup * root_display_group_;
  ToolManager * tool_manager_;
  ViewManager * view_manager_;
  properties::Property * global_options_;
  properties::TfFrameProperty * fixed_frame_property_;
  properties::StatusList * global_status_;
  properties::IntProperty * fps_property_;
  RenderPanel * render_panel_;
  std::chrono::system_clock::time_point wall_clock_begin_;
  rclcpp::Time ros_time_begin_;
  std::chrono::system_clock::duration wall_clock_elapsed_;
  rclcpp::Duration ros_time_elapsed_;
  rviz_common::properties::ColorProperty * background_color_property_;
  std::chrono::nanoseconds time_update_timer_;
  std::chrono::nanoseconds frame_update_timer_;
  std::shared_ptr<rviz_common::interaction::HandlerManagerIface> handler_manager_;
  std::shared_ptr<rviz_common::interaction::SelectionManagerIface> selection_manager_;
  std::shared_ptr<rviz_common::interaction::ViewPickerIface> view_picker_;
  uint32_t render_requested_;
  uint64_t frame_count_;
  WindowManagerInterface * window_manager_;
  FrameManager * frame_manager_;
  OgreRenderQueueClearer * ogre_render_queue_clearer_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::JumpHandler::SharedPtr clock_jump_handler_;

private Q_SLOTS:
  void updateFixedFrame();
  void updateBackgroundColor();
  void updateFps();

private:
  DisplayFactory * display_factory_;
  VisualizationManagerSpinSomePrivate * private_;
  uint32_t default_visibility_bit_;
  BitAllocator visibility_bit_allocator_;
  QString help_path_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  // NOTE: No std::thread executor_thread_ — this is the key difference.
  //       The executor is driven by spin_some() inside onUpdate() instead.
  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rviz_common::transformation::TransformationManager * transformation_manager_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__VISUALIZATION_MANAGER_SPIN_SOME_HPP_
