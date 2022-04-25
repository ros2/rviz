/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "rviz_common/visualization_manager.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <OgreCamera.h>
#include <OgreLight.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreViewport.h>

#include <QApplication>  // NOLINT: cpplint cannot handle include order here
#include <QCursor>  // NOLINT: cpplint cannot handle include order here
#include <QTimer>  // NOLINT: cpplint cannot handle include order here
#include <QWindow>  // NOLINT: cpplint cannot handle include order here

// #include "tf/transform_listener.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rviz_common/display.hpp"
#include "./display_factory.hpp"
#include "rviz_common/display_group.hpp"
#include "./displays_panel.hpp"
#include "frame_manager.hpp"
#include "rviz_common/load_resource.hpp"
// #include "./ogre_helpers/ogre_render_queue_clearer.hpp"
// #include "./ogre_helpers/qt_ogre_render_window.hpp"
// #include "./ogre_helpers/render_system.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/properties/status_list.hpp"
#include "rviz_common/properties/tf_frame_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/interaction/handler_manager.hpp"
#include "rviz_common/interaction/handler_manager_iface.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/interaction/selection_manager_iface.hpp"
#include "rviz_common/interaction/view_picker.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/tool_manager.hpp"
// #include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"
// #include "./viewport_mouse_event.hpp"

// #include "rviz/window_manager_interface.h"

namespace rviz_common
{

using rviz_common::properties::ColorProperty;
using rviz_common::properties::IntProperty;
using rviz_common::properties::PropertyTreeModel;
using rviz_common::properties::StatusList;
using rviz_common::properties::StatusProperty;
using rviz_common::properties::TfFrameProperty;
using rviz_common::interaction::HandlerManager;
using rviz_common::interaction::SelectionManager;
using rviz_common::interaction::ViewPicker;
using rviz_common::interaction::M_Picked;

// helper class needed to display an icon besides "Global Options"
class IconizedProperty : public rviz_common::properties::Property
{
public:
  IconizedProperty(
    const QString & name = QString(),
    const QVariant default_value = QVariant(),
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0)
  : Property(name, default_value, description, parent, changed_slot, receiver) {}
  virtual QVariant getViewData(int column, int role) const
  {
    return (column == 0 && role == Qt::DecorationRole) ?
           icon_ : Property::getViewData(column, role);
  }
  void setIcon(const QIcon & icon) {icon_ = icon;}

private:
  QIcon icon_;
};

class VisualizationManagerPrivate
{
public:
  // ros::CallbackQueue threaded_queue_;
  // boost::thread_group threaded_queue_threads_;
  // ros::NodeHandle update_nh_;
  // ros::NodeHandle threaded_nh_;
  std::mutex render_mutex_;
};

VisualizationManager::VisualizationManager(
  RenderPanel * render_panel,
  ros_integration::RosNodeAbstractionIface::WeakPtr ros_node_abstraction,
  WindowManagerInterface * wm, rclcpp::Clock::SharedPtr clock)
: ogre_root_(Ogre::Root::getSingletonPtr()),
  update_timer_(0),
  shutting_down_(false),
  render_panel_(render_panel),
  time_update_timer_(0.0f),
  frame_update_timer_(0.0f),
  render_requested_(1),
  frame_count_(0),
  window_manager_(wm),
  clock_(clock),
  private_(new VisualizationManagerPrivate),
  executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
  rviz_ros_node_(ros_node_abstraction)
{
  // visibility_bit_allocator_ is listed after default_visibility_bit_
  // (and thus initialized later be default):
  default_visibility_bit_ = visibility_bit_allocator_.allocBit();

  transformation_manager_ = new transformation::TransformationManager(rviz_ros_node_, clock_);
  connect(transformation_manager_, SIGNAL(configChanged()), this, SIGNAL(configChanged()));

  frame_manager_ = new FrameManager(clock, transformation_manager_->getCurrentTransformer());
  connect(
    transformation_manager_,
    SIGNAL(transformerChanged(std::shared_ptr<rviz_common::transformation::FrameTransformer>)),
    frame_manager_,
    SLOT(setTransformerPlugin(std::shared_ptr<rviz_common::transformation::FrameTransformer>)));

// TODO(wjwwood): is this needed?
#if 0
  render_panel->setAutoRender(false);
#endif

  // scene_manager_ = ogre_root_->createSceneManager(Ogre::ST_GENERIC);

// TODO(wjwwood): is this needed?
#if 0
  rviz::RenderSystem::RenderSystem::get()->prepareOverlays(scene_manager_);
#endif

  root_display_group_ = new DisplayGroup();
  root_display_group_->setName("root");
  display_property_tree_model_ = new PropertyTreeModel(root_display_group_);
  display_property_tree_model_->setDragDropClass("display");
  connect(display_property_tree_model_, SIGNAL(configChanged()), this, SIGNAL(configChanged()));

  tool_manager_ = new ToolManager(this);
  connect(tool_manager_, SIGNAL(configChanged()), this, SIGNAL(configChanged()));
  connect(tool_manager_, SIGNAL(toolChanged(Tool*)), this, SLOT(onToolChanged(Tool*)));

  view_manager_ = new ViewManager(this);
  view_manager_->setRenderPanel(render_panel_);
  connect(view_manager_, SIGNAL(configChanged()), this, SIGNAL(configChanged()));

  IconizedProperty * ip =
    new IconizedProperty("Global Options", QVariant(), "", root_display_group_);
  ip->setIcon(loadPixmap("package://rviz_common/icons/options.png"));
  global_options_ = ip;

  fixed_frame_property_ = new TfFrameProperty(
    "Fixed Frame", "base_link",
    "Frame into which all data is transformed before being displayed.",
    global_options_, frame_manager_, false,
    SLOT(updateFixedFrame()), this);

  background_color_property_ = new ColorProperty(
    "Background Color", QColor(48, 48, 48),
    "Background color for the 3D view.",
    global_options_, SLOT(updateBackgroundColor()), this);

  fps_property_ = new IntProperty(
    "Frame Rate", 30,
    "RViz will try to render this many frames per second.",
    global_options_, SLOT(updateFps()), this);

  root_display_group_->initialize(this);   // only initialize() a Display
                                           // after its sub-properties are created.
  root_display_group_->setEnabled(true);

  updateFixedFrame();
  updateBackgroundColor();

  global_status_ = new StatusList("Global Status", root_display_group_);
  global_status_->setReadOnly(true);

  rviz_rendering::MaterialManager::createDefaultColorMaterials();

  handler_manager_ = std::make_shared<HandlerManager>();
  selection_manager_ = std::make_shared<SelectionManager>(this);
  view_picker_ = std::make_shared<ViewPicker>(this);

  rcl_jump_threshold_t jump_threshold;
  jump_threshold.on_clock_change = true;
  // Disable forward jump callbacks
  jump_threshold.min_forward.nanoseconds = 0;
  // Anything backwards is a jump
  jump_threshold.min_backward.nanoseconds = -1;
  clock_jump_handler_ = clock_->create_jump_callback(
    nullptr, std::bind(
      &VisualizationManager::onTimeJump, this,
      std::placeholders::_1), jump_threshold);

  connect(this, SIGNAL(timeJumped()), this, SLOT(resetTime()));

  executor_->add_node(rviz_ros_node_.lock()->get_raw_node());
// TODO(wjwwood): redo with executors?
#if 0
  private_->threaded_queue_threads_.create_thread(
    std::bind(&VisualizationManager::threadedQueueThreadFunc, this));
#endif

  display_factory_ = new DisplayFactory();

// TODO(wjwwood): move this to rviz_rendering somewhere?
#if 0
  ogre_render_queue_clearer_ = new OgreRenderQueueClearer();
  Ogre::Root::getSingletonPtr()->addFrameListener(ogre_render_queue_clearer_);
#endif

  update_timer_ = new QTimer;
  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));
}

VisualizationManager::~VisualizationManager()
{
  delete update_timer_;

  shutting_down_ = true;
#if 0
  private_->threaded_queue_threads_.join_all();
#endif

  delete display_property_tree_model_;
  delete tool_manager_;
  delete display_factory_;
  delete frame_manager_;
  delete private_;
  delete transformation_manager_;

#if 0
  Ogre::Root::getSingletonPtr()->removeFrameListener(ogre_render_queue_clearer_);
  delete ogre_render_queue_clearer_;
#endif
}

void VisualizationManager::initialize()
{
  emitStatusUpdate("Initializing managers.");

  view_manager_->initialize();
  selection_manager_->initialize();
  view_picker_->initialize();
  tool_manager_->initialize();

  last_update_ros_time_ = clock_->now();
  last_update_wall_time_ = std::chrono::system_clock::now();
}

#if 0
ros::CallbackQueueInterface * VisualizationManager::getThreadedQueue()
{
  return &private_->threaded_queue_;
}
#endif

void VisualizationManager::lockRender()
{
  private_->render_mutex_.lock();
}

void VisualizationManager::unlockRender()
{
  private_->render_mutex_.unlock();
}

ros_integration::RosNodeAbstractionIface::WeakPtr
VisualizationManager::getRosNodeAbstraction() const
{
  return rviz_ros_node_;
}

#if 0
ros::CallbackQueueInterface * VisualizationManager::getUpdateQueue()
{
  return ros::getGlobalCallbackQueue();
}
#endif

void VisualizationManager::startUpdate()
{
  float interval = 1000.0 / static_cast<float>(fps_property_->getInt());
  update_timer_->start(interval);
}

void VisualizationManager::stopUpdate()
{
  update_timer_->stop();
}

void VisualizationManager::queueRender()
{
  render_requested_ = 1;
}

WindowManagerInterface * VisualizationManager::getWindowManager() const
{
  return window_manager_;
}

FrameManagerIface * VisualizationManager::getFrameManager() const
{
  return frame_manager_;
}

uint64_t VisualizationManager::getFrameCount() const
{
  return frame_count_;
}

DisplayFactory * VisualizationManager::getDisplayFactory() const
{
  return display_factory_;
}

properties::PropertyTreeModel * VisualizationManager::getDisplayTreeModel() const
{
  return display_property_tree_model_;
}

DisplayGroup * VisualizationManager::getRootDisplayGroup() const
{
  return root_display_group_;
}

uint32_t VisualizationManager::getDefaultVisibilityBit() const
{
  return default_visibility_bit_;
}

BitAllocator * VisualizationManager::visibilityBits()
{
  return &visibility_bit_allocator_;
}

void VisualizationManager::onUpdate()
{
  const auto wall_now = std::chrono::system_clock::now();
  const auto wall_diff = wall_now - last_update_wall_time_;
  const uint64_t wall_dt = std::chrono::duration_cast<std::chrono::nanoseconds>(wall_diff).count();
  const auto ros_now = clock_->now();
  const uint64_t ros_dt = ros_now.nanoseconds() - last_update_ros_time_.nanoseconds();
  last_update_ros_time_ = ros_now;
  last_update_wall_time_ = wall_now;

  if (ros_dt < 0.0) {
    resetTime();
  }

  executor_->spin_some(std::chrono::milliseconds(10));

  Q_EMIT preUpdate();

  frame_manager_->update();

  root_display_group_->update(wall_dt, ros_dt);

  if (nullptr != view_manager_) {
    view_manager_->update(wall_dt, ros_dt);
  }

  time_update_timer_ += wall_dt;

  if (time_update_timer_ > 0.1f) {
    time_update_timer_ = 0.0f;

    updateTime();
  }

  frame_update_timer_ += wall_dt;

  if (frame_update_timer_ > 1.0f) {
    frame_update_timer_ = 0.0f;

    updateFrames();
  }

  selection_manager_->update();

  if (tool_manager_->getCurrentTool()) {
    tool_manager_->getCurrentTool()->update(wall_dt, ros_dt);
  }

  if (view_manager_ &&
    view_manager_->getCurrent() &&
    view_manager_->getCurrent()->getCamera())
  {
    using rviz_rendering::RenderWindowOgreAdapter;
    RenderWindowOgreAdapter::getDirectionalLight(render_panel_->getRenderWindow())->setDirection(
      view_manager_->getCurrent()->getCamera()->getDerivedDirection());
  }

  frame_count_++;
  if (render_requested_ || wall_diff > std::chrono::milliseconds(10)) {
    render_requested_ = 0;
    std::lock_guard<std::mutex> lock(private_->render_mutex_);
    ogre_root_->renderOneFrame();
  }
}

void VisualizationManager::onTimeJump(const rcl_time_jump_t & jump)
{
  if (jump.clock_change == RCL_ROS_TIME_ACTIVATED ||
    jump.clock_change == RCL_ROS_TIME_DEACTIVATED)
  {
    RVIZ_COMMON_LOG_WARNING("Detected time source change. Resetting RViz.");
    Q_EMIT timeJumped();
  } else if (jump.delta.nanoseconds < 0) {
    RVIZ_COMMON_LOG_WARNING_STREAM(
      "Detected jump back in time. Resetting RViz.");
    Q_EMIT timeJumped();
  }
}

void VisualizationManager::updateTime()
{
  rclcpp::Clock clock;  // TODO(wjwwood): replace with clock attached to node for ROS Time
  if (ros_time_begin_.nanoseconds() == 0) {
    ros_time_begin_ = clock_->now();
  }

  ros_time_elapsed_ = (clock_->now() - ros_time_begin_).nanoseconds();

  if (wall_clock_begin_.time_since_epoch().count() == 0) {
    wall_clock_begin_ = std::chrono::system_clock::now();
  }

  wall_clock_elapsed_ = std::chrono::system_clock::now() - wall_clock_begin_;
}

void VisualizationManager::updateFrames()
{
  // Check the fixed frame to see if it's ok
  std::string error;
  if (frame_manager_->frameHasProblems(getFixedFrame().toStdString(), error)) {
    if (!frame_manager_->anyTransformationDataAvailable()) {
      // fixed_prop->setToWarn();
      std::stringstream ss;
      ss << "No tf data.  Actual error: " << error;
      global_status_->setStatus(
        StatusProperty::Warn, "Fixed Frame", QString::fromStdString(ss.str()));
    } else {
      // fixed_prop->setToError();
      global_status_->setStatus(
        StatusProperty::Error, "Fixed Frame", QString::fromStdString(error));
    }
  } else {
    // fixed_prop->setToOK();
    global_status_->setStatus(StatusProperty::Ok, "Fixed Frame", "OK");
  }
}

Ogre::SceneManager * VisualizationManager::getSceneManager() const
{
  using rviz_rendering::RenderWindowOgreAdapter;
  return RenderWindowOgreAdapter::getSceneManager(render_panel_->getRenderWindow());
}

RenderPanel * VisualizationManager::getRenderPanel() const
{
  return render_panel_;
}

void VisualizationManager::resetTime()
{
  root_display_group_->reset();
  frame_manager_->clear();

  ros_time_begin_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  wall_clock_begin_ = std::chrono::system_clock::time_point();

  queueRender();
}

std::shared_ptr<rviz_common::interaction::HandlerManagerIface>
VisualizationManager::getHandlerManager() const
{
  return handler_manager_;
}

std::shared_ptr<rviz_common::interaction::SelectionManagerIface>
VisualizationManager::getSelectionManager() const
{
  return selection_manager_;
}

std::shared_ptr<rviz_common::interaction::ViewPickerIface>
VisualizationManager::getViewPicker() const
{
  return view_picker_;
}

ToolManager * VisualizationManager::getToolManager() const
{
  return tool_manager_;
}

ViewManager * VisualizationManager::getViewManager() const
{
  return view_manager_;
}

transformation::TransformationManager * VisualizationManager::getTransformationManager()
{
  return transformation_manager_;
}

void VisualizationManager::addDisplay(Display * display, bool enabled)
{
  root_display_group_->addDisplay(display);
  display->initialize(this);
  display->setEnabled(enabled);
}

void VisualizationManager::removeAllDisplays()
{
  root_display_group_->removeAllDisplays();
}

void VisualizationManager::emitStatusUpdate(const QString & message)
{
  Q_EMIT statusUpdate(message);
}

void VisualizationManager::load(const Config & config)
{
  stopUpdate();

  emitStatusUpdate("Creating displays");
  root_display_group_->load(config);

  emitStatusUpdate("Creating tools");
  tool_manager_->load(config.mapGetChild("Tools"));

  emitStatusUpdate("Creating views");
  view_manager_->load(config.mapGetChild("Views"));

  emitStatusUpdate("Loading transformation");
  transformation_manager_->load(config.mapGetChild("Transformation"));

  startUpdate();
}

void VisualizationManager::save(Config config) const
{
  root_display_group_->save(config);
  tool_manager_->save(config.mapMakeChild("Tools"));
  view_manager_->save(config.mapMakeChild("Views"));
  transformation_manager_->save(config.mapMakeChild("Transformation"));
}

Display * VisualizationManager::createDisplay(
  const QString & class_lookup_name,
  const QString & name,
  bool enabled)
{
  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  Display * new_display = root_display_group_->createDisplay(class_lookup_name);
  addDisplay(new_display, enabled);
  new_display->setName(name);
  QApplication::restoreOverrideCursor();
  return new_display;
}

double VisualizationManager::getWallClock()
{
  return std::chrono::duration_cast<std::chrono::duration<double>>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

double VisualizationManager::getROSTime()
{
  return frame_manager_->getTime().nanoseconds() / 1e9;
}

double VisualizationManager::getWallClockElapsed()
{
  return std::chrono::duration_cast<std::chrono::duration<double>>(wall_clock_elapsed_).count();
}

double VisualizationManager::getROSTimeElapsed()
{
  return static_cast<double>(ros_time_elapsed_) / 1e9;
}

void VisualizationManager::updateBackgroundColor()
{
  using rviz_rendering::RenderWindowOgreAdapter;
  auto ogre_color = rviz_common::properties::qtToOgre(background_color_property_->getColor());
  rviz_rendering::RenderWindowOgreAdapter::setBackgroundColor(
    render_panel_->getRenderWindow(),
    &ogre_color);

  queueRender();
}

void VisualizationManager::updateFps()
{
  if (update_timer_->isActive()) {
    startUpdate();
  }
}

void VisualizationManager::handleMouseEvent(const ViewportMouseEvent & vme)
{
  // process pending mouse events
  Tool * current_tool = tool_manager_->getCurrentTool();

  int flags = 0;
  if (current_tool) {
    ViewportMouseEvent _vme = vme;
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    QWindow * window = vme.panel->windowHandle();
    if (window) {
      double pixel_ratio = window->devicePixelRatio();
      _vme.x = static_cast<int>(pixel_ratio * _vme.x);
      _vme.y = static_cast<int>(pixel_ratio * _vme.y);
      _vme.last_x = static_cast<int>(pixel_ratio * _vme.last_x);
      _vme.last_y = static_cast<int>(pixel_ratio * _vme.last_y);
    }
#endif
    flags = current_tool->processMouseEvent(_vme);
    vme.panel->setCursor(current_tool->getCursor());
    vme.panel->getRenderWindow()->setCursor(current_tool->getCursor());
  } else {
    vme.panel->setCursor(QCursor(Qt::ArrowCursor));
  }

  if (flags & Tool::Render) {
    queueRender();
  }

  if (flags & Tool::Finished) {
    tool_manager_->setCurrentTool(tool_manager_->getDefaultTool());
  }
}

void VisualizationManager::handleChar(QKeyEvent * event, RenderPanel * panel)
{
  tool_manager_->handleChar(event, panel);
}

void VisualizationManager::threadedQueueThreadFunc()
{
  // TODO(wjwwood): redo with executors
#if 0
  while (!shutting_down_) {
    private_->threaded_queue_.callOne(ros::WallDuration(0.1));
  }
#endif
}

void VisualizationManager::notifyConfigChanged()
{
  Q_EMIT configChanged();
}

void VisualizationManager::onToolChanged(Tool * tool)
{
  Q_UNUSED(tool);
}

void VisualizationManager::updateFixedFrame()
{
  QString frame = fixed_frame_property_->getFrame();

  frame_manager_->setFixedFrame(frame.toStdString());
  root_display_group_->setFixedFrame(frame);
}

QString VisualizationManager::getFixedFrame() const
{
  return fixed_frame_property_->getFrame();
}

void VisualizationManager::setFixedFrame(const QString & frame)
{
  fixed_frame_property_->setValue(frame);
}

void VisualizationManager::setStatus(const QString & message)
{
  emitStatusUpdate(message);
}

void VisualizationManager::setHelpPath(const QString & help_path)
{
  help_path_ = help_path;
}

QString VisualizationManager::getHelpPath() const
{
  return help_path_;
}

rclcpp::Clock::SharedPtr VisualizationManager::getClock()
{
  return clock_;
}

}  // namespace rviz_common
