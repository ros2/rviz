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

#include "rviz_common/view_controller.hpp"

#include <string>

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <QFont>  // NOLINT: cpplint cannot handle include order here
#include <QKeyEvent>  // NOLINT: cpplint cannot handle include order here

#include "rviz_rendering/render_window.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace rviz_common
{

using properties::BoolProperty;
using properties::FloatProperty;

ViewController::ViewController()
: context_(nullptr),
  camera_(nullptr),
  is_active_(false),
  type_property_(nullptr)
{
  near_clip_property_ = new FloatProperty(
    "Near Clip Distance", 0.01f,
    "Anything closer to the camera than this threshold will not get rendered.",
    this, SLOT(updateNearClipDistance()));
  near_clip_property_->setMin(0.001f);
  near_clip_property_->setMax(10000);

  stereo_enable_ = new BoolProperty(
    "Enable Stereo Rendering", true,
    "Render the main view in stereo if supported."
    "  On Linux this requires a recent version of Ogre and"
    " an NVIDIA Quadro card with 3DVision glasses.",
    this, SLOT(updateStereoProperties()));
  stereo_eye_swap_ = new BoolProperty(
    "Swap Stereo Eyes", false,
    "Swap eyes if the monitor shows the left eye on the right.",
    stereo_enable_, SLOT(updateStereoProperties()), this);
  stereo_eye_separation_ = new FloatProperty(
    "Stereo Eye Separation", 0.06f,
    "Distance between eyes for stereo rendering.",
    stereo_enable_, SLOT(updateStereoProperties()), this);
  stereo_focal_distance_ = new FloatProperty(
    "Stereo Focal Distance", 1.0f,
    "Distance from eyes to screen.  For stereo rendering.",
    stereo_enable_, SLOT(updateStereoProperties()), this);
  invert_z_ = new BoolProperty(
    "Invert Z Axis", false,
    "Invert camera's Z axis for Z-down environments/models.",
    this, SLOT(updateStereoProperties()));
}

void ViewController::initialize(DisplayContext * context)
{
  context_ = context;

  static int count = 0;
  camera_ = context_->getSceneManager()->createCamera(
    "ViewControllerCamera" + std::to_string(count++));
  context_->getSceneManager()->getRootSceneNode()->attachObject(camera_);

  setValue(formatClassId(getClassId()));
  setReadOnly(true);

  // Do subclass initialization.
  onInitialize();

  cursor_ = getDefaultCursor();

  standard_cursors_[Default] = getDefaultCursor();
  standard_cursors_[Rotate2D] = makeIconCursor("package://rviz_common/icons/rotate.svg");
  standard_cursors_[Rotate3D] = makeIconCursor("package://rviz_common/icons/rotate_cam.svg");
  standard_cursors_[MoveXY] = makeIconCursor("package://rviz_common/icons/move2d.svg");
  standard_cursors_[MoveZ] = makeIconCursor("package://rviz_common/icons/move_z.svg");
  standard_cursors_[Zoom] = makeIconCursor("package://rviz_common/icons/zoom.svg");
  standard_cursors_[Crosshair] = makeIconCursor("package://rviz_common/icons/crosshair.svg");

  updateNearClipDistance();
  updateStereoProperties();

  // TODO(wjwwood): replace this with a call to the rviz_rendering::RenderWindow or similar
  //                Until then, stereo is disabled per default
  // if (!RenderSystem::get()->isStereoSupported()) {
  stereo_enable_->setBool(false);
  stereo_enable_->hide();
  // }

  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (ros_node_abstraction) {
    auto node = ros_node_abstraction->get_raw_node();
    reset_time_srv_ = node->create_service<std_srvs::srv::Empty>(
      ros_node_abstraction->get_node_name() + "/reset_time",
      std::bind(
        &ViewController::resetService, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }
}

ViewController::~ViewController()
{
  context_->getSceneManager()->destroyCamera(camera_);
}

QString ViewController::formatClassId(const QString & class_id)
{
  QStringList id_parts = class_id.split("/");
  if (id_parts.size() != 2) {
    // Should never happen with pluginlib class ids, which are
    // formatted like "package_name/class_name".  Not worth crashing
    // over though.
    return class_id;
  } else {
    return id_parts[1] + " (" + id_parts[0] + ")";
  }
}

QVariant ViewController::getViewData(int column, int role) const
{
  if (role == Qt::ForegroundRole) {
    return QVariant();
  }

  if (is_active_) {
    switch (role) {
      case Qt::FontRole:
        {
          QFont font;
          font.setBold(true);
          return font;
        }
    }
  }
  return Property::getViewData(column, role);
}

Qt::ItemFlags ViewController::getViewFlags(int column) const
{
  if (is_active_) {
    return Property::getViewFlags(column);
  } else {
    return Property::getViewFlags(column) | Qt::ItemIsDragEnabled;
  }
}

void ViewController::activate()
{
  is_active_ = true;
  onActivate();
}

void ViewController::update(float dt, float ros_dt)
{
  (void) dt;
  (void) ros_dt;
}

void ViewController::handleMouseEvent(ViewportMouseEvent & evt)
{
  (void) evt;
}

void ViewController::emitConfigChanged()
{
  Q_EMIT configChanged();
}

void ViewController::load(const Config & config)
{
  // Load the name by hand.
  QString name;
  if (config.mapGetString("Name", &name)) {
    setName(name);
  }
  // Load all sub-properties the same way the base class does.
  Property::load(config);
}

void ViewController::save(Config config) const
{
  config.mapSetValue("Class", getClassId());
  config.mapSetValue("Name", getName());

  Property::save(config);
}

void ViewController::handleKeyEvent(QKeyEvent * event, RenderPanel * panel)
{
  if (event->key() == Qt::Key_F && context_->getViewPicker()) {
    QPoint mouse_rel_panel = panel->mapFromGlobal(QCursor::pos());
    Ogre::Vector3 point_rel_world;  // output of get3DPoint().
    if (
      context_->getViewPicker()->get3DPoint(
        panel,
        mouse_rel_panel.x(), mouse_rel_panel.y(),
        point_rel_world))
    {
      lookAt(point_rel_world);
    }
  }

  if (event->key() == Qt::Key_Z) {
    reset();
  }

  if (event->key() == Qt::Key_R) {
    resetTime();
  }
}

void ViewController::resetService(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  const std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  resetTime();
}

void ViewController::resetTime()
{
  rviz_common::VisualizationManager * vis_manager =
    dynamic_cast<rviz_common::VisualizationManager *>(context_);

  if (vis_manager != nullptr) {
    vis_manager->resetTime();
  }
}

void ViewController::setCursor(CursorType cursor_type)
{
  cursor_ = standard_cursors_[cursor_type];
}

void ViewController::lookAt(float x, float y, float z)
{
  Ogre::Vector3 point(x, y, z);
  lookAt(point);
}

void ViewController::mimic(ViewController * source_view)
{
  (void) source_view;
}

void ViewController::transitionFrom(ViewController * previous_view)
{
  (void) previous_view;
}

Ogre::Camera * ViewController::getCamera() const
{
  return camera_;
}

QString ViewController::getClassId() const
{
  return class_id_;
}

void ViewController::setClassId(const QString & class_id)
{
  class_id_ = class_id;
}

bool ViewController::isActive() const
{
  return is_active_;
}

QCursor ViewController::getCursor()
{
  return cursor_;
}

FocalPointStatus ViewController::getFocalPointStatus()
{
  return FocalPointStatus();
}

void ViewController::setStatus(const QString & message)
{
  if (context_) {
    context_->setStatus(message);
  }
}

void ViewController::updateNearClipDistance()
{
  camera_->setNearClipDistance(near_clip_property_->getFloat());
}

void ViewController::updateStereoProperties()
{
  if (stereo_enable_->getBool()) {
    float focal_dist = stereo_focal_distance_->getFloat();
    float eye_sep = stereo_eye_swap_->getBool() ?
      -stereo_eye_separation_->getFloat() :
      stereo_eye_separation_->getFloat();
    camera_->setFrustumOffset(0.5f * eye_sep, 0.0f);
    camera_->setFocalLength(focal_dist);
    stereo_eye_swap_->show();
    stereo_eye_separation_->show();
    stereo_focal_distance_->show();
  } else {
    camera_->setFrustumOffset(0.0f, 0.0f);
    camera_->setFocalLength(1.0f);
    stereo_eye_swap_->hide();
    stereo_eye_separation_->hide();
    stereo_focal_distance_->hide();
  }
}

void ViewController::updateInvertZAxis()
{
  // We don't seem to need to do anything here.
}

void ViewController::onInitialize()
{
}

void ViewController::onActivate()
{
}

void ViewController::setCursor(QCursor cursor)
{
  cursor_ = cursor;
}

}  // namespace rviz_common
