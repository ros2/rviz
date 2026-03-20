// Copyright (c) 2008, Willow Garage, Inc.
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
// INTERRUPTION) IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

// This file is a copy of visualization_manager.cpp as it existed BEFORE
// commit 9716a8a ("Use a thread to run ROS executor").
//
// Differences from visualization_manager.cpp (post-9716a8a):
//   - No #include <thread>.
//   - Constructor does NOT launch an executor thread.
//   - Destructor does NOT cancel/join an executor thread.
//   - onUpdate() calls executor_->spin_some(std::chrono::milliseconds(10))
//     before emitting preUpdate() — the OLD pattern.

#include "rviz_common/visualization_manager_spin_some.hpp"

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
#include <QKeyEvent>  // NOLINT: cpplint cannot handle include order here
#include <QString>  // NOLINT: cpplint cannot handle include order here
#include <QTimer>  // NOLINT: cpplint cannot handle include order here
#include <QWindow>  // NOLINT: cpplint cannot handle include order here

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
#include "rviz_common/view_manager.hpp"


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
class IconizedPropertySpinSome : public rviz_common::properties::Property
{
public:
  IconizedPropertySpinSome(
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

class VisualizationManagerSpinSomePrivate
{
public:
  std::mutex render_mutex_;
};

VisualizationManagerSpinSome::VisualizationManagerSpinSome(
  RenderPanel * render_panel,
  ros_integration::RosNodeAbstractionIface::WeakPtr ros_node_abstraction,
  WindowManagerInterface * wm, rclcpp::Clock::SharedPtr clock)
: ogre_root_(Ogre::Root::getSingletonPtr()),
  update_timer_(0),
  shutting_down_(false),
  render_panel_(render_panel),
  wall_clock_elapsed_(0),
  ros_time_elapsed_(0, 0),
  time_update_timer_(0),
  frame_update_timer_(0),
  render_requested_(1),
  frame_count_(0),
  window_manager_(wm),
  clock_(clock),
  private_(new VisualizationManagerSpinSomePrivate()),
  executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
  rviz_ros_node_(ros_node_abstraction)
{
  // visibility_bit_allocator_ is listed after default_visibility_bit_
  // (and thus initialized later by default):
  default_visibility_bit_ = visibility_bit_allocator_.allocBit();

  transformation_manager_ = new transformation::TransformationManager(rviz_ros_node_, clock_);
  connect(transformation_manager_, SIGNAL(configChanged()), this, SIGNAL(configChanged()));

  frame_manager_ = new FrameManager(clock, transformation_manager_->getCurrentTransformer());
  connect(
    transformation_manager_,
    SIGNAL(transformerChanged(std::shared_ptr<rviz_common::transformation::FrameTransformer>)),
    frame_manager_,
    SLOT(setTransformerPlugin(std::shared_ptr<rviz_common::transformation::FrameTransformer>)));

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

  IconizedPropertySpinSome * ip =
    new IconizedPropertySpinSome("Global Options", QVariant(), "", root_display_group_);
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

  root_display_group_->initialize(this);
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
  jump_threshold.min_forward.nanoseconds = 0;
  jump_threshold.min_backward.nanoseconds = -1;
  clock_jump_handler_ = clock_->create_jump_callback(
    nullptr, std::bind(
      &VisualizationManagerSpinSome::onTimeJump, this,
      std::placeholders::_1), jump_threshold);

  connect(this, SIGNAL(timeJumped()), this, SLOT(resetTime()));

  // OLD pattern: node is added to executor but NO thread is started.
  // The executor is driven by spin_some() inside onUpdate().
  executor_->add_node(rviz_ros_node_.lock()->get_raw_node());

  display_factory_ = new DisplayFactory();
  update_timer_ = new QTimer;
  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));
}

VisualizationManagerSpinSome::~VisualizationManagerSpinSome()
{
  delete update_timer_;

  shutting_down_ = true;
  // OLD pattern: no executor_->cancel() or thread join needed.

  delete display_property_tree_model_;
  delete tool_manager_;
  delete display_factory_;
  delete frame_manager_;
  delete private_;
  delete transformation_manager_;
}

void VisualizationManagerSpinSome::initialize()
{
  emitStatusUpdate("Initializing managers.");

  view_manager_->initialize();
  selection_manager_->initialize();
  view_picker_->initialize();
  tool_manager_->initialize();

  last_update_ros_time_ = clock_->now();
  last_update_wall_time_ = std::chrono::system_clock::now();
}

void VisualizationManagerSpinSome::lockRender()
{
  private_->render_mutex_.lock();
}

void VisualizationManagerSpinSome::unlockRender()
{
  private_->render_mutex_.unlock();
}

ros_integration::RosNodeAbstractionIface::WeakPtr
VisualizationManagerSpinSome::getRosNodeAbstraction() const
{
  return rviz_ros_node_;
}

void VisualizationManagerSpinSome::startUpdate()
{
  float interval = 1000.0 / static_cast<float>(fps_property_->getInt());
  update_timer_->start(interval);
}

void VisualizationManagerSpinSome::stopUpdate()
{
  update_timer_->stop();
}

void VisualizationManagerSpinSome::queueRender()
{
  render_requested_ = 1;
}

WindowManagerInterface * VisualizationManagerSpinSome::getWindowManager() const
{
  return window_manager_;
}

FrameManagerIface * VisualizationManagerSpinSome::getFrameManager() const
{
  return frame_manager_;
}

uint64_t VisualizationManagerSpinSome::getFrameCount() const
{
  return frame_count_;
}

DisplayFactory * VisualizationManagerSpinSome::getDisplayFactory() const
{
  return display_factory_;
}

properties::PropertyTreeModel * VisualizationManagerSpinSome::getDisplayTreeModel() const
{
  return display_property_tree_model_;
}

DisplayGroup * VisualizationManagerSpinSome::getRootDisplayGroup() const
{
  return root_display_group_;
}

uint32_t VisualizationManagerSpinSome::getDefaultVisibilityBit() const
{
  return default_visibility_bit_;
}

BitAllocator * VisualizationManagerSpinSome::visibilityBits()
{
  return &visibility_bit_allocator_;
}

void VisualizationManagerSpinSome::onUpdate()
{
  const std::chrono::time_point<std::chrono::system_clock> wall_now =
    std::chrono::system_clock::now();
  const std::chrono::nanoseconds wall_dt =
    std::chrono::duration_cast<std::chrono::nanoseconds>(wall_now - last_update_wall_time_);
  const auto ros_now = clock_->now();
  const auto ros_dt = std::chrono::nanoseconds(
      std::lround(ros_now.nanoseconds() - last_update_ros_time_.nanoseconds()));
  last_update_ros_time_ = ros_now;
  last_update_wall_time_ = wall_now;

  if (ros_dt < std::chrono::nanoseconds(0)) {
    resetTime();
  }

  // OLD pattern (removed in commit 9716a8a):
  // The executor is driven here, on the Qt main thread, once per update tick.
  executor_->spin_some(std::chrono::milliseconds(10));

  Q_EMIT preUpdate();

  frame_manager_->update();

  root_display_group_->update(wall_dt, ros_dt);

  if (nullptr != view_manager_) {
    view_manager_->update(wall_dt, ros_dt);
  }

  time_update_timer_ += wall_dt;

  if (time_update_timer_ > std::chrono::nanoseconds(0)) {
    time_update_timer_ = std::chrono::nanoseconds(0);
    updateTime();
  }

  frame_update_timer_ += wall_dt;

  if (frame_update_timer_ > std::chrono::nanoseconds(1)) {
    frame_update_timer_ = std::chrono::nanoseconds(0);
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
    rviz_rendering::RenderWindowOgreAdapter::setDirectionalLightDirection(
      render_panel_->getRenderWindow(),
      view_manager_->getCurrent()->getCamera()->getDerivedDirection());
  }

  frame_count_++;
  if (render_requested_ || wall_dt > std::chrono::milliseconds(10)) {
    render_requested_ = 0;
    std::lock_guard<std::mutex> lock(private_->render_mutex_);
    ogre_root_->renderOneFrame();
  }
}

void VisualizationManagerSpinSome::onTimeJump(const rcl_time_jump_t & jump)
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

void VisualizationManagerSpinSome::updateTime()
{
  if (ros_time_begin_.nanoseconds() == 0) {
    ros_time_begin_ = rviz_ros_node_.lock()->get_raw_node()->get_clock()->now();
  }

  ros_time_elapsed_ = rviz_ros_node_.lock()->get_raw_node()->get_clock()->now() - ros_time_begin_;

  if (wall_clock_begin_.time_since_epoch().count() == 0) {
    wall_clock_begin_ = std::chrono::system_clock::now();
  }

  wall_clock_elapsed_ = std::chrono::system_clock::now() - wall_clock_begin_;
}

void VisualizationManagerSpinSome::updateFrames()
{
  std::string error;
  if (frame_manager_->frameHasProblems(getFixedFrame().toStdString(), error)) {
    if (!frame_manager_->anyTransformationDataAvailable()) {
      std::stringstream ss;
      ss << "No tf data.  Actual error: " << error;
      global_status_->setStatus(
        StatusProperty::Warn, "Fixed Frame", QString::fromStdString(ss.str()));
    } else {
      global_status_->setStatus(
        StatusProperty::Error, "Fixed Frame", QString::fromStdString(error));
    }
  } else {
    global_status_->setStatus(StatusProperty::Ok, "Fixed Frame", "OK");
  }
}

Ogre::SceneManager * VisualizationManagerSpinSome::getSceneManager() const
{
  using rviz_rendering::RenderWindowOgreAdapter;
  return RenderWindowOgreAdapter::getSceneManager(render_panel_->getRenderWindow());
}

RenderPanel * VisualizationManagerSpinSome::getRenderPanel() const
{
  return render_panel_;
}

void VisualizationManagerSpinSome::resetTime()
{
  root_display_group_->reset();
  frame_manager_->clear();

  ros_time_begin_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  wall_clock_begin_ = std::chrono::system_clock::time_point();

  queueRender();
}

std::shared_ptr<rviz_common::interaction::HandlerManagerIface>
VisualizationManagerSpinSome::getHandlerManager() const
{
  return handler_manager_;
}

std::shared_ptr<rviz_common::interaction::SelectionManagerIface>
VisualizationManagerSpinSome::getSelectionManager() const
{
  return selection_manager_;
}

std::shared_ptr<rviz_common::interaction::ViewPickerIface>
VisualizationManagerSpinSome::getViewPicker() const
{
  return view_picker_;
}

ToolManager * VisualizationManagerSpinSome::getToolManager() const
{
  return tool_manager_;
}

ViewManager * VisualizationManagerSpinSome::getViewManager() const
{
  return view_manager_;
}

transformation::TransformationManager *
VisualizationManagerSpinSome::getTransformationManager()
{
  return transformation_manager_;
}

void VisualizationManagerSpinSome::addDisplay(Display * display, bool enabled)
{
  root_display_group_->addDisplay(display);
  display->initialize(this);
  display->setEnabled(enabled);
}

void VisualizationManagerSpinSome::removeAllDisplays()
{
  root_display_group_->removeAllDisplays();
}

void VisualizationManagerSpinSome::emitStatusUpdate(const QString & message)
{
  Q_EMIT statusUpdate(message);
}

void VisualizationManagerSpinSome::load(const Config & config)
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

void VisualizationManagerSpinSome::save(Config config) const
{
  root_display_group_->save(config);
  tool_manager_->save(config.mapMakeChild("Tools"));
  view_manager_->save(config.mapMakeChild("Views"));
  transformation_manager_->save(config.mapMakeChild("Transformation"));
}

Display * VisualizationManagerSpinSome::createDisplay(
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

double VisualizationManagerSpinSome::getWallClock()
{
  return std::chrono::duration_cast<std::chrono::duration<double>>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

double VisualizationManagerSpinSome::getROSTime()
{
  return frame_manager_->getTime().nanoseconds() / 1e9;
}

double VisualizationManagerSpinSome::getWallClockElapsed()
{
  return std::chrono::duration_cast<std::chrono::duration<double>>(wall_clock_elapsed_).count();
}

double VisualizationManagerSpinSome::getROSTimeElapsed()
{
  return ros_time_elapsed_.seconds();
}

void VisualizationManagerSpinSome::updateBackgroundColor()
{
  using rviz_rendering::RenderWindowOgreAdapter;
  auto ogre_color = rviz_common::properties::qtToOgre(background_color_property_->getColor());
  rviz_rendering::RenderWindowOgreAdapter::setBackgroundColor(
    render_panel_->getRenderWindow(),
    &ogre_color);

  queueRender();
}

void VisualizationManagerSpinSome::updateFps()
{
  if (update_timer_->isActive()) {
    startUpdate();
  }
}

void VisualizationManagerSpinSome::handleMouseEvent(const ViewportMouseEvent & vme)
{
  Tool * current_tool = tool_manager_->getCurrentTool();

  int flags = 0;
  if (current_tool) {
    ViewportMouseEvent _vme = vme;

    QWindow * window = vme.panel->windowHandle();
    if (window) {
      double pixel_ratio = window->devicePixelRatio();
      _vme.x = static_cast<int>(pixel_ratio * _vme.x);
      _vme.y = static_cast<int>(pixel_ratio * _vme.y);
      _vme.last_x = static_cast<int>(pixel_ratio * _vme.last_x);
      _vme.last_y = static_cast<int>(pixel_ratio * _vme.last_y);
    }

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

void VisualizationManagerSpinSome::handleChar(QKeyEvent * event, RenderPanel * panel)
{
  if (event->key() == Qt::Key_Escape) {
    Q_EMIT escapePressed();
  }

  int flags = tool_manager_->handleChar(event, panel);

  if (flags & Tool::Render) {
    queueRender();
  }

  if (flags & Tool::Finished) {
    tool_manager_->setCurrentTool(tool_manager_->getDefaultTool());
  }
}

void VisualizationManagerSpinSome::notifyConfigChanged()
{
  Q_EMIT configChanged();
}

void VisualizationManagerSpinSome::onToolChanged(Tool * tool)
{
  Q_UNUSED(tool);
}

void VisualizationManagerSpinSome::updateFixedFrame()
{
  QString frame = fixed_frame_property_->getFrame();

  frame_manager_->setFixedFrame(frame.toStdString());
  root_display_group_->setFixedFrame(frame);
}

QString VisualizationManagerSpinSome::getFixedFrame() const
{
  return fixed_frame_property_->getFrame();
}

void VisualizationManagerSpinSome::setFixedFrame(const QString & frame)
{
  fixed_frame_property_->setValue(frame);
}

void VisualizationManagerSpinSome::setStatus(const QString & message)
{
  emitStatusUpdate(message);
}

void VisualizationManagerSpinSome::setHelpPath(const QString & help_path)
{
  help_path_ = help_path;
}

QString VisualizationManagerSpinSome::getHelpPath() const
{
  return help_path_;
}

rclcpp::Clock::SharedPtr VisualizationManagerSpinSome::getClock()
{
  return clock_;
}

}  // namespace rviz_common
