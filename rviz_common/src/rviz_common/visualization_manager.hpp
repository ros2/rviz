/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_COMMON__VISUALIZATION_MANAGER_HPP_
#define RVIZ_COMMON__VISUALIZATION_MANAGER_HPP_

#include <chrono>
#include <deque>
#include <memory>

#include "rclcpp/time.hpp"
#include "tf2_ros/transform_listener.h"

#include "./bit_allocator.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"

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

class VisualizationManagerPrivate;

/// The VisualizationManager class is the central manager class of rviz.
/**
 * It holds all the Displays, Tools, ViewControllers, and other managers.
 *
 * It keeps the current view controller for the main render window.
 * It has a timer which calls update() on all the displays.
 * It creates and holds pointers to the other manager objects:
 * SelectionManager, FrameManager, the PropertyManagers, and Ogre::SceneManager.
 *
 * The "protected" members should probably all be "private", as
 * VisualizationManager is not intended to be subclassed.
 */
class VisualizationManager : public DisplayContext
{
  Q_OBJECT

public:
  /// Constructor.
  /**
   * Creates managers and sets up global properties.
   * \param render_panel a pointer to the main render panel widget of the app.
   * \param wm a pointer to the window manager (which is really just a
   *   VisualizationFrame, the top-level container widget of rviz).
   * \param tf a pointer to tf::TransformListener which will be internally used by FrameManager.
   */
  explicit VisualizationManager(
    RenderPanel * render_panel,
    WindowManagerInterface * wm = 0,
    std::shared_ptr<tf2_ros::TransformListener> tf = nullptr,
    std::shared_ptr<tf2_ros::Buffer> buffer = nullptr
  );

  /// Destructor.
  /**
   * Stops the update of timers and destroys all displays, tools, and managers.
   */
  virtual ~VisualizationManager();

  /// Do initialization that was not done in constructor.
  /**
   * Initializes tool manager, view manager, selection manager.
   */
  void initialize();

  /// Start timers.
  /**
   * Creates and starts the update and idle timers, both set to 30Hz (33ms).
   */
  void startUpdate();

  /// Stop the update timers.
  /**
   * No Displays will be updated and no ROS callbacks will be called during
   * this period.
   */
  void stopUpdate();

  /// Create and add a display to this panel, by class lookup name.
  /**
   * \param class_lookup_name "lookup name" of the Display subclass, for pluginlib.
   *   Should be of the form "packagename/displaynameofclass", like "rviz/Image".
   * \param name The name of this display instance shown on the GUI, like "Left arm camera".
   * \param enabled Whether to start enabled
   * \return A pointer to the new display.
   */
  Display * createDisplay(const QString & class_lookup_name, const QString & name, bool enabled);

  /// Add a display to be managed by this panel.
  /**
   * \param display The display to be added.
   */
  void addDisplay(Display * display, bool enabled);

  /// Remove and delete all displays.
  void removeAllDisplays();

  /// Load the properties of each Display and most editable rviz data.
  /**
   * This is what is called when loading a "*.rviz" file.
   *
   * \param config The Config object to read from.  Expected to be a Config::Map type.
   * \see save()
   */
  void load(const Config & config);

  /// Save the properties of each Display and most editable rviz data.
  /**
   * This is what is called when saving a "*.vcg" file.
   *
   * \param config The object to write to.
   * \see loadDisplayConfig()
   */
  void save(Config config) const;

  /// Return the fixed frame name.
  /**
   * \see setFixedFrame()
   */
  QString getFixedFrame() const override;

  /// Set the coordinate frame we should be transforming all fixed data into.
  /**
   * \param frame The frame name; must match the frame name broadcast of tf.
   * \see getFixedFrame()
   */
  void setFixedFrame(const QString & frame);

  /// Return the Ogre::SceneManager used for the main RenderPanel.
  Ogre::SceneManager * getSceneManager() const override;

  /// Return the main RenderPanel.
  RenderPanel * getRenderPanel() const;

  /// Return the wall clock time, in seconds since 1970.
  double getWallClock();

  /// Return the ROS time, in seconds.
  double getROSTime();

  /// Return the wall clock time in seconds since the last reset.
  double getWallClockElapsed();

  /// Return the ROS time in seconds since the last reset.
  double getROSTimeElapsed();

  /// Handle a single key event for a given RenderPanel.
  /**
   * If the key is Escape, switches to the default Tool (via getDefaultTool()).
   * All other key events are passed to the current Tool (via getCurrentTool()).
   */
  void handleChar(QKeyEvent * event, RenderPanel * panel) override;

  /// Handle a mouse event.
  /**
   * This just copies the given event into an event queue.
   * The events in the queue are processed by onUpdate() which is called from
   * the main thread by a timer every 33ms.
   */
  void handleMouseEvent(const ViewportMouseEvent & event) override;

  /// Resets the wall and ROS elapsed time to zero and calls resetDisplays().
  void resetTime();

  /// Return a pointer to the SelectionManager.
  rviz_common::selection::SelectionManager * getSelectionManager() const override;

  /// Return a pointer to the ToolManager.
  ToolManager * getToolManager() const override;

  /// Return a pointer to the ViewManager.
  ViewManager * getViewManager() const override;

  /// Lock a mutex to delay calls to Ogre::Root::renderOneFrame().
  void lockRender();

  /// Unlock a mutex, allowing calls to Ogre::Root::renderOneFrame().
  void unlockRender();

  /// Queues a render.
  /**
   * Multiple calls before a render happens will only cause a single render.
   *
   * \note This function can be called from any thread.
   */
  void queueRender() override;

  /// Return the window manager, if any.
  WindowManagerInterface * getWindowManager() const override;

  void addNodeToMainExecutor(rclcpp::Node::SharedPtr) override;

  void removeNodeFromMainExecutor(rclcpp::Node::SharedPtr) override;

#if 0
  /**
   * @brief Return a CallbackQueue using a different thread than the main GUI one.
   */
  ros::CallbackQueueInterface * getThreadedQueue();
#endif

  /// Return the FrameManager instance.
  FrameManager * getFrameManager() const override;

  /// Return the current value of the frame count.
  /**
   * The frame count is just a number which increments each time a
   * frame is rendered.
   * This lets clients check if a new frame has been rendered since the last
   * time they did something.
   */
  uint64_t getFrameCount() const override;

  /// Notify this VisualizationManager that something about its display configuration has changed.
  void notifyConfigChanged();

// TODO(wjwwood): reenable when display factory is fixed
#if 0
  /// Return a factory for creating Display subclasses based on a class id string.
  DisplayFactory * getDisplayFactory() const override;
#endif

  /// Return the display tree model.
  properties::PropertyTreeModel * getDisplayTreeModel() const;

  /// Emit statusUpdate() signal with the given message.
  void emitStatusUpdate(const QString & message);

  /// Return the root display group
  DisplayGroup * getRootDisplayGroup() const override;

  /// Get the default visibility bit.
  uint32_t getDefaultVisibilityBit() const override;

  BitAllocator * visibilityBits() override;

  void setStatus(const QString & message) override;

  virtual void setHelpPath(const QString & help_path);

  virtual QString getHelpPath() const;

Q_SIGNALS:

  /// Emitted before updating all Displays.
  void preUpdate();

  /// Emitted whenever the display configuration changes.
  void configChanged();

  /// Emitted during file-loading and initialization to indicate progress.
  void statusUpdate(const QString & message);

protected Q_SLOTS:
  /// Call update() on all managed objects.
  /**
   * This is the central place where update() is called on most rviz objects.
   * Display objects, the FrameManager, the current ViewController, the
   * SelectionManager, PropertyManager.
   * Also calls ros::spinOnce(), so any callbacks on the global CallbackQueue
   * get called from here as well.
   *
   * It is called at 30Hz from the update timer.
   */
  void onUpdate();

  void onToolChanged(Tool *);

protected:
  void updateTime();

  void updateFrames();

  void createColorMaterials();

  void threadedQueueThreadFunc();

  /// Ogre Root.
  Ogre::Root * ogre_root_;

  /// Update timer; Display::update is called on each display whenever this timer fires.
  QTimer * update_timer_;

  /// Update stopwatch; Stores how long it's been since the last update.
  rclcpp::Time last_update_ros_time_;
  std::chrono::system_clock::time_point last_update_wall_time_;

  volatile bool shutting_down_;

  properties::PropertyTreeModel * display_property_tree_model_;
  DisplayGroup * root_display_group_;

  ToolManager * tool_manager_;
  ViewManager * view_manager_;

  properties::Property * global_options_;
  /// Frame to transform fixed data into.
  properties::TfFrameProperty * fixed_frame_property_;
  properties::StatusList * global_status_;
  properties::IntProperty * fps_property_;

  RenderPanel * render_panel_;

  std::chrono::system_clock::time_point wall_clock_begin_;
  rclcpp::Time ros_time_begin_;
  std::chrono::system_clock::duration wall_clock_elapsed_;
  // TODO(wjwwood): replace with rclcpp::Duration when available
  uint64_t ros_time_elapsed_;

  rviz_common::properties::ColorProperty * background_color_property_;

  float time_update_timer_;
  float frame_update_timer_;

  rviz_common::selection::SelectionManager * selection_manager_;

  uint32_t render_requested_;
  uint64_t frame_count_;

  WindowManagerInterface * window_manager_;

  FrameManager * frame_manager_;

  OgreRenderQueueClearer * ogre_render_queue_clearer_;

private Q_SLOTS:
  void updateFixedFrame();
  void updateBackgroundColor();
  void updateFps();

private:
  DisplayFactory * display_factory_;
  VisualizationManagerPrivate * private_;
  uint32_t default_visibility_bit_;
  BitAllocator visibility_bit_allocator_;
  QString help_path_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__VISUALIZATION_MANAGER_HPP_
