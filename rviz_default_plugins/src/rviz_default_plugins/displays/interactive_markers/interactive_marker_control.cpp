/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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
 *     * Neither the name of the copyright holder nor the names of its
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
#include <limits>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <OgreViewport.h>
#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgrePass.h>
#include <OgreMaterial.h>
#include <OgreEntity.h>
#include <OgreSubEntity.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/handler_manager.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/window_manager_interface.hpp"
#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/line.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rviz_default_plugins/displays/marker/markers/shape_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/arrow_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/line_list_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/line_strip_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/points_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/text_view_facing_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/mesh_resource_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/triangle_list_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/marker_base.hpp"

#include "rviz_default_plugins/displays/interactive_markers/interactive_marker.hpp"
#include "rviz_default_plugins/displays/interactive_markers/interactive_marker_control.hpp"

static const float kNoHighlightValue = 0.0f;
static const float kActiveHightlightValue = 0.5f;
static const float kHoverHighlightValue = 0.3f;

namespace rviz_default_plugins
{
namespace displays
{

InteractiveMarkerControl::InteractiveMarkerControl(
  rviz_common::DisplayContext * context,
  Ogre::SceneNode * reference_node,
  InteractiveMarker * parent)
: mouse_dragging_(false),
  drag_viewport_(nullptr),
  dragging_in_place_event_(nullptr),
  context_(context),
  reference_node_(reference_node),
  control_frame_node_(reference_node_->createChildSceneNode()),
  markers_node_(reference_node_->createChildSceneNode()),
  parent_(parent),
  rotation_(0),
  grab_point_in_reference_frame_(0, 0, 0),
  interaction_enabled_(false),
  visible_(true),
  view_facing_(false),
  mouse_down_(false),
  show_visual_aids_(false),
  line_(new rviz_rendering::Line(context->getSceneManager(), control_frame_node_))
{
  line_->setVisible(false);
}

void InteractiveMarkerControl::makeMarkers(
  const visualization_msgs::msg::InteractiveMarkerControl & message)
{
  for (const auto & message_marker : message.markers) {
    markers::MarkerBase::SharedPtr marker;

    // create a marker with the given type
    switch (message_marker.type) {
      case visualization_msgs::msg::Marker::CUBE:
      case visualization_msgs::msg::Marker::CYLINDER:
      case visualization_msgs::msg::Marker::SPHERE:
        {
          marker.reset(new markers::ShapeMarker(nullptr, context_, markers_node_));
        }
        break;

      case visualization_msgs::msg::Marker::ARROW:
        {
          marker.reset(new markers::ArrowMarker(nullptr, context_, markers_node_));
        }
        break;

      case visualization_msgs::msg::Marker::LINE_STRIP:
        {
          marker.reset(new markers::LineStripMarker(nullptr, context_, markers_node_));
        }
        break;
      case visualization_msgs::msg::Marker::LINE_LIST:
        {
          marker.reset(new markers::LineListMarker(nullptr, context_, markers_node_));
        }
        break;
      case visualization_msgs::msg::Marker::SPHERE_LIST:
      case visualization_msgs::msg::Marker::CUBE_LIST:
      case visualization_msgs::msg::Marker::POINTS:
        {
          auto points_marker = std::make_shared<markers::PointsMarker>(
            nullptr, context_, markers_node_);
          points_markers_.push_back(points_marker);
          marker = points_marker;
        }
        break;
      case visualization_msgs::msg::Marker::TEXT_VIEW_FACING:
        {
          marker.reset(new markers::TextViewFacingMarker(nullptr, context_, markers_node_));
        }
        break;
      case visualization_msgs::msg::Marker::MESH_RESOURCE:
        {
          marker.reset(new markers::MeshResourceMarker(nullptr, context_, markers_node_));
        }
        break;

      case visualization_msgs::msg::Marker::TRIANGLE_LIST:
        {
          marker.reset(new markers::TriangleListMarker(nullptr, context_, markers_node_));
        }
        break;
      default:
        RVIZ_COMMON_LOG_ERROR_STREAM("Unknown marker type: " << message_marker.type);
        break;
    }

    auto marker_msg = std::make_shared<visualization_msgs::msg::Marker>(message_marker);

    if (marker_msg->header.frame_id.empty()) {
      // Put Marker into fixed frame, so the constructor does not apply any tf transform.
      // This effectively discards any tf information in the Marker and interprets its pose
      // as relative to the Interactive Marker.
      marker_msg->header.frame_id = context_->getFrameManager()->getFixedFrame();
      marker->setMessage(marker_msg);
    } else {
      marker->setMessage(marker_msg);
      // The marker will set its position relative to the fixed frame,
      // but we have attached it our own scene node, so we will have to
      // correct for that.
      marker->setPosition(markers_node_->convertWorldToLocalPosition(marker->getPosition()));
      marker->setOrientation(
        markers_node_->convertWorldToLocalOrientation(marker->getOrientation()));
    }
    marker->setInteractiveObject(shared_from_this());

    addHighlightPass(marker->getMaterials());

    markers_.push_back(marker);
  }
}

InteractiveMarkerControl::~InteractiveMarkerControl()
{
  context_->getSceneManager()->destroySceneNode(control_frame_node_);
  context_->getSceneManager()->destroySceneNode(markers_node_);

  if (view_facing_) {
    context_->getSceneManager()->removeListener(this);
  }
}

void InteractiveMarkerControl::processMessage(
  const visualization_msgs::msg::InteractiveMarkerControl & message)
{
  name_ = message.name;
  description_ = QString::fromStdString(message.description);
  interaction_mode_ = message.interaction_mode;
  always_visible_ = message.always_visible;
  orientation_mode_ = message.orientation_mode;

  control_orientation_ = Ogre::Quaternion(
    message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z);
  control_orientation_.normalise();

  bool new_view_facingness =
    (message.orientation_mode == visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING);
  if (new_view_facingness != view_facing_) {
    if (new_view_facingness) {
      context_->getSceneManager()->addListener(this);
    } else {
      context_->getSceneManager()->removeListener(this);
    }
    view_facing_ = new_view_facingness;
  }

  independent_marker_orientation_ = message.independent_marker_orientation;

  // highlight_passes_ have raw pointers into the markers_, so must
  // clear them at the same time.
  highlight_passes_.clear();
  markers_.clear();
  points_markers_.clear();

  // Initially, the pose of this marker's node and the interactive
  // marker are identical, but that may change.
  control_frame_node_->setPosition(parent_->getPosition());
  markers_node_->setPosition(parent_->getPosition());

  if (orientation_mode_ == visualization_msgs::msg::InteractiveMarkerControl::INHERIT) {
    control_frame_node_->setOrientation(parent_->getOrientation());
    markers_node_->setOrientation(parent_->getOrientation());
  } else {
    control_frame_node_->setOrientation(Ogre::Quaternion::IDENTITY);
    markers_node_->setOrientation(Ogre::Quaternion::IDENTITY);
  }

  makeMarkers(message);

  status_msg_ = description_ + " ";

  Ogre::Vector3 control_dir = control_orientation_.xAxis() * 10000.0;
  line_->setPoints(control_dir, -1 * control_dir);
  line_->setVisible(false);

  // Create our own custom cursor
  switch (interaction_mode_) {
    case visualization_msgs::msg::InteractiveMarkerControl::NONE:
      cursor_ = rviz_common::getDefaultCursor();
      break;
    case visualization_msgs::msg::InteractiveMarkerControl::MENU:
      cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/menu.svg");
      status_msg_ += "<b>Left-Click:</b> Show menu.";
      break;
    case visualization_msgs::msg::InteractiveMarkerControl::BUTTON:
      cursor_ = rviz_common::getDefaultCursor();
      status_msg_ += "<b>Left-Click:</b> Activate. ";
      break;
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS:
      cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/move1d.svg");
      status_msg_ += "<b>Left-Click:</b> Move. ";
      break;
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE:
      cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/move2d.svg");
      status_msg_ += "<b>Left-Click:</b> Move. ";
      break;
    case visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS:
      cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/rotate.svg");
      status_msg_ += "<b>Left-Click:</b> Rotate. ";
      break;
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE:
      cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/moverotate.svg");
      status_msg_ += "<b>Left-Click:</b> Move / Rotate. ";
      break;
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D:
      cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/move2d.svg");
      status_msg_ += "<b>Left-Click:</b> Move X/Y. <b>Shift + Left-Click / Left-Click + "
        "Wheel:</b> Move Z. ";
      break;
    case visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D:
      cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/rotate.svg");
      status_msg_ += "<b>Left-Click:</b> Rotate around X/Y. <b>Shift-Left-Click:</b> "
        "Rotate around Z. ";
      break;
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D:
      cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/moverotate.svg");
      status_msg_ += "<b>Left-Click:</b> Move X/Y. <b>Shift + Left-Click / Left-Click + "
        "Wheel:</b> Move Z. <b>Ctrl + Left-Click:</b> Rotate around X/Y. <b>Ctrl + Shift + "
        "Left-Click:</b> Rotate around Z. ";
      break;
    default:
      break;
  }

  if (parent_->hasMenu() &&
    interaction_mode_ != visualization_msgs::msg::InteractiveMarkerControl::MENU)
  {
    status_msg_ += "<b>Right-Click:</b> Show context menu.";
  }

  // It's not clear to me why this one setOrientation() call needs to
  // be here and not above makeMarkers() with the other
  // setOrientation() calls, but it works correctly when here and
  // incorrectly when there.  Sorry. -hersh
  if (orientation_mode_ == visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING &&
    independent_marker_orientation_)
  {
    markers_node_->setOrientation(parent_->getOrientation());
  }

  enableInteraction(context_->getHandlerManager()->getInteractionEnabled());
}

// This is an Ogre::SceneManager::Listener function, and is configured
// to be called only if this is a VIEW_FACING control.
void InteractiveMarkerControl::preFindVisibleObjects(
  Ogre::SceneManager * source,
  Ogre::SceneManager::IlluminationRenderStage irs,
  Ogre::Viewport * v)
{
  (void)source;
  (void)irs;
  updateControlOrientationForViewFacing(v);
}

void InteractiveMarkerControl::updateControlOrientationForViewFacing(Ogre::Viewport * v)
{
  Ogre::Quaternion x_view_facing_rotation = control_orientation_.xAxis().getRotationTo(
    v->getCamera()->getDerivedDirection());

  // rotate so z axis is up
  Ogre::Vector3 z_axis_2 = x_view_facing_rotation * control_orientation_.zAxis();
  Ogre::Quaternion align_yz_rotation = z_axis_2.getRotationTo(v->getCamera()->getDerivedUp());

  // rotate
  Ogre::Quaternion rotate_around_x = Ogre::Quaternion(
    rotation_, v->getCamera()->getDerivedDirection());

  Ogre::Quaternion rotation = reference_node_->convertWorldToLocalOrientation(
    rotate_around_x * align_yz_rotation * x_view_facing_rotation);

  control_frame_node_->setOrientation(rotation);

  if (!independent_marker_orientation_) {
    markers_node_->setOrientation(rotation);
    // we need to refresh the node manually, since the scene manager will only do this one frame
    // later otherwise
    markers_node_->_update(true, false);
  }
}

bool InteractiveMarkerControl::getVisible()
{
  return visible_ || always_visible_;
}

void InteractiveMarkerControl::setVisible(bool visible)
{
  visible_ = visible;

  if (always_visible_) {
    markers_node_->setVisible(visible_);
  } else {
    markers_node_->setVisible(interaction_enabled_ && visible_);
  }
}

void InteractiveMarkerControl::update()
{
  if (mouse_dragging_ && dragging_in_place_event_) {
    handleMouseMovement(*dragging_in_place_event_);
  }
}

void InteractiveMarkerControl::enableInteraction(bool enable)
{
  if (mouse_down_) {
    return;
  }

  interaction_enabled_ = enable;
  setVisible(visible_);
  if (!enable) {
    setHighlight(kNoHighlightValue);
  }
}

void InteractiveMarkerControl::interactiveMarkerPoseChanged(
  Ogre::Vector3 int_marker_position, Ogre::Quaternion int_marker_orientation)
{
  control_frame_node_->setPosition(int_marker_position);
  markers_node_->setPosition(int_marker_position);

  switch (orientation_mode_) {
    case visualization_msgs::msg::InteractiveMarkerControl::INHERIT:
      control_frame_node_->setOrientation(int_marker_orientation);
      markers_node_->setOrientation(control_frame_node_->getOrientation());
      break;

    case visualization_msgs::msg::InteractiveMarkerControl::FIXED:
      {
        control_frame_node_->setOrientation(
          Ogre::Quaternion(rotation_, control_orientation_.xAxis()));
        markers_node_->setOrientation(control_frame_node_->getOrientation());
        break;
      }

    case visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING:
      if (drag_viewport_) {
        updateControlOrientationForViewFacing(drag_viewport_);
      }
      if (independent_marker_orientation_) {
        markers_node_->setOrientation(int_marker_orientation);
      }
      break;
    default:
      break;
  }
}

Ogre::Vector3 InteractiveMarkerControl::closestPointOnLineToPoint(
  const Ogre::Vector3 & line_start,
  const Ogre::Vector3 & line_dir,
  const Ogre::Vector3 & test_point)
{
  // Find closest point on projected ray to mouse point
  // Math: if P is the start of the ray, v is the direction vector of
  //       the ray (not normalized), and X is the test point, then the
  //       closest point on the line to X is given by:
  //
  //               (X-P).v
  //       P + v * -------
  //                 v.v
  //       where "." is the dot product.
  double factor = (test_point - line_start).dotProduct(line_dir) / line_dir.dotProduct(line_dir);
  Ogre::Vector3 closest_point = line_start + line_dir * factor;
  return closest_point;
}

void InteractiveMarkerControl::rotate(Ogre::Ray & mouse_ray)
{
  Ogre::Vector3 intersection_3d;
  Ogre::Vector2 intersection_2d;
  float ray_t;

  Ogre::Vector3 rotation_axis = control_frame_orientation_at_mouse_down_ *
    control_orientation_.xAxis();

  // Find rotation_center = 3D point closest to grab_point_ which is
  // on the rotation axis, relative to the reference frame.
  Ogre::Vector3 rotation_center = closestPointOnLineToPoint(
    control_frame_node_->getPosition(),
    rotation_axis,
    grab_point_in_reference_frame_);

  // Find intersection of mouse_ray with plane centered at rotation_center.
  if (intersectSomeYzPlane(
      mouse_ray,
      rotation_center,
      control_frame_orientation_at_mouse_down_,
      intersection_3d,
      intersection_2d,
      ray_t))
  {
    // Not that efficient, but reduces code duplication...
    rotate(intersection_3d);
  }
}

void InteractiveMarkerControl::rotate(const Ogre::Vector3 & cursor_in_reference_frame)
{
  Ogre::Vector3 rotation_axis = control_frame_orientation_at_mouse_down_ *
    control_orientation_.xAxis();

  // Find rotation_center = 3D point closest to grab_point_ which is
  // on the rotation axis, relative to the reference frame.
  Ogre::Vector3 rotation_center = closestPointOnLineToPoint(
    control_frame_node_->getPosition(),
    rotation_axis,
    grab_point_in_reference_frame_);

  Ogre::Vector3 grab_rel_center = grab_point_in_reference_frame_ - rotation_center;
  Ogre::Vector3 center_to_cursor = cursor_in_reference_frame - rotation_center;
  Ogre::Vector3 center_to_cursor_radial = center_to_cursor -
    center_to_cursor.dotProduct(rotation_axis) * rotation_axis;

  Ogre::Quaternion orientation_change_since_mouse_down =
    grab_rel_center.getRotationTo(center_to_cursor_radial, rotation_axis);

  Ogre::Radian rot;
  Ogre::Vector3 axis;
  orientation_change_since_mouse_down.ToAngleAxis(rot, axis);
  // Quaternion::ToAngleAxis() always gives a positive angle.  The
  // axis it emits (in this case) will either be equal to
  // rotation_axis or will be the negative of it.  Doing the
  // dot-product then gives either 1.0 or -1.0, which is just what
  // we need to get the sign for our angle.
  Ogre::Radian rotation_since_mouse_down = rot * axis.dotProduct(rotation_axis);

  rotation_ = rotation_at_mouse_down_ + rotation_since_mouse_down;
  parent_->setPose(
    parent_->getPosition(),
    orientation_change_since_mouse_down * parent_orientation_at_mouse_down_,
    name_);
}

Ogre::Ray InteractiveMarkerControl::getMouseRayInReferenceFrame(
  const rviz_common::ViewportMouseEvent & event, int x, int y)
{
  auto viewport =
    rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
  float width = viewport->getActualWidth() - 1;
  float height = viewport->getActualHeight() - 1;

  Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay(
    (x + .5) / width, (y + .5) / height);

  // convert ray into reference frame
  mouse_ray.setOrigin(reference_node_->convertWorldToLocalPosition(mouse_ray.getOrigin()));
  mouse_ray.setDirection(
    reference_node_->convertWorldToLocalOrientation(
      Ogre::Quaternion::IDENTITY) * mouse_ray.getDirection());

  return mouse_ray;
}

void InteractiveMarkerControl::beginRelativeMouseMotion(
  const rviz_common::ViewportMouseEvent & event)
{
  mouse_x_at_drag_begin_ = event.x;
  mouse_y_at_drag_begin_ = event.y;
  modifiers_at_drag_begin_ = event.modifiers;

  mouse_ray_at_drag_begin_ = getMouseRayInReferenceFrame(event, event.x, event.y);

  // ensure direction is unit vector
  mouse_ray_at_drag_begin_.setDirection(mouse_ray_at_drag_begin_.getDirection().normalisedCopy());
}

bool InteractiveMarkerControl::getRelativeMouseMotion(
  const rviz_common::ViewportMouseEvent & event, int & dx, int & dy)
{
  dx = event.x - mouse_x_at_drag_begin_;
  dy = event.y - mouse_y_at_drag_begin_;
  if (dx == 0 && dy == 0) {
    return false;
  }

  QCursor::setPos(
    mouse_x_at_drag_begin_ + mouse_relative_to_absolute_x_,
    mouse_y_at_drag_begin_ + mouse_relative_to_absolute_y_);
  return true;
}

void InteractiveMarkerControl::rotateXYRelative(const rviz_common::ViewportMouseEvent & event)
{
  int dx;
  int dy;

  if (!getRelativeMouseMotion(event, dx, dy)) {
    return;
  }

  static const double MOUSE_SCALE = 2 * 3.14 / 300;  // 300 pixels = 360deg
  Ogre::Radian rx(dx * MOUSE_SCALE);
  Ogre::Radian ry(dy * MOUSE_SCALE);

  auto viewport =
    rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
  Ogre::Quaternion up_rot(rx, viewport->getCamera()->getRealUp());
  Ogre::Quaternion right_rot(ry, viewport->getCamera()->getRealRight());

  parent_->setPose(parent_->getPosition(), up_rot * right_rot * parent_->getOrientation(), name_);
}

void InteractiveMarkerControl::rotateZRelative(const rviz_common::ViewportMouseEvent & event)
{
  int dx;
  int dy;

  getRelativeMouseMotion(event, dx, dy);
  if (std::abs(dy) > std::abs(dx)) {
    dx = dy;
  }
  if (dx == 0) {
    return;
  }

  static const double MOUSE_SCALE = 2 * 3.14 / 300;  // 300 pixels = 360deg
  Ogre::Radian rx(dx * MOUSE_SCALE);

  auto viewport =
    rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
  Ogre::Quaternion rot(rx, viewport->getCamera()->getRealDirection());

  parent_->setPose(parent_->getPosition(), rot * parent_->getOrientation(), name_);
}

void InteractiveMarkerControl::moveZAxisRelative(const rviz_common::ViewportMouseEvent & event)
{
  int dx;
  int dy;

  getRelativeMouseMotion(event, dx, dy);
  if (std::abs(dx) > std::abs(dy)) {
    dy = -dx;
  }
  if (dy == 0) {
    return;
  }

  double distance = -dy * mouse_z_scale_;
  Ogre::Vector3 delta = distance * mouse_ray_at_drag_begin_.getDirection();

  parent_->setPose(parent_->getPosition() + delta, parent_->getOrientation(), name_);

  parent_position_at_mouse_down_ = parent_->getPosition();
}

void InteractiveMarkerControl::moveZAxisWheel(const rviz_common::ViewportMouseEvent & event)
{
  // wheel_delta is in 1/8 degree and usually jumps 15 degrees at a time
  static const double WHEEL_TO_PIXEL_SCALE = (1.0 / 8.0) * (2.0 / 15.0);  // 2 pixels per click

  double distance = event.wheel_delta * WHEEL_TO_PIXEL_SCALE;
  Ogre::Vector3 delta = distance * mouse_ray_at_drag_begin_.getDirection();

  parent_->setPose(parent_->getPosition() + delta, parent_->getOrientation(), name_);

  parent_position_at_mouse_down_ = parent_->getPosition();
}

void InteractiveMarkerControl::moveViewPlane(
  Ogre::Ray & mouse_ray, const rviz_common::ViewportMouseEvent & event)
{
  // find plane on which mouse is moving
  auto viewport =
    rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
  Ogre::Plane plane(
    viewport->getCamera()->getRealDirection(),
    grab_point_in_reference_frame_);

  // find intersection of mouse with the plane
  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(plane);
  if (!intersection.first) {
    return;
  }
  Ogre::Vector3 mouse_position_on_plane = mouse_ray.getPoint(intersection.second);

  // move parent so grab position relative to parent coincides with new mouse position.
  parent_->setPose(
    mouse_position_on_plane - grab_point_in_reference_frame_ + parent_position_at_mouse_down_,
    parent_->getOrientation(),
    name_);
}

void InteractiveMarkerControl::movePlane(Ogre::Ray & mouse_ray)
{
  if (orientation_mode_ == visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING &&
    drag_viewport_)
  {
    updateControlOrientationForViewFacing(drag_viewport_);
  }

  Ogre::Vector3 intersection_3d;
  Ogre::Vector2 intersection_2d;
  float ray_t;

  if (intersectSomeYzPlane(
      mouse_ray,
      grab_point_in_reference_frame_,
      control_frame_node_->getOrientation(),
      intersection_3d,
      intersection_2d,
      ray_t))
  {
    parent_->setPose(
      intersection_3d - grab_point_in_reference_frame_ + parent_position_at_mouse_down_,
      parent_->getOrientation(),
      name_);
  }
}

void InteractiveMarkerControl::movePlane(const Ogre::Vector3 & cursor_position_in_reference_frame)
{
  if (orientation_mode_ == visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING &&
    drag_viewport_)
  {
    updateControlOrientationForViewFacing(drag_viewport_);
  }

  Ogre::Vector3 plane_normal = control_frame_node_->getOrientation() * control_orientation_.xAxis();
  Ogre::Vector3 displacement = cursor_position_in_reference_frame - grab_point_in_reference_frame_;
  Ogre::Vector3 displacement_on_plane = displacement -
    displacement.dotProduct(plane_normal) * plane_normal;

  // set position of parent to parent_position_at_mouse_down_ + displacement_on_plane.
  parent_->setPose(
    parent_position_at_mouse_down_ + displacement_on_plane, parent_->getOrientation(), name_);
}

void InteractiveMarkerControl::worldToScreen(
  const Ogre::Vector3 & pos_rel_reference,
  const Ogre::Viewport * viewport,
  Ogre::Vector2 & screen_pos)
{
  Ogre::Vector3 world_pos = reference_node_->convertLocalToWorldPosition(pos_rel_reference);

  const Ogre::Camera * cam = viewport->getCamera();
  Ogre::Vector3 homogeneous_screen_position = cam->getProjectionMatrix() *
    (cam->getViewMatrix() * world_pos);

  double half_width = viewport->getActualWidth() / 2.0;
  double half_height = viewport->getActualHeight() / 2.0;

  screen_pos.x = half_width + (half_width * homogeneous_screen_position.x) - .5;
  screen_pos.y = half_height + (half_height * -homogeneous_screen_position.y) - .5;
}

bool InteractiveMarkerControl::findClosestPoint(
  const Ogre::Ray & target_ray, const Ogre::Ray & mouse_ray, Ogre::Vector3 & closest_point)
{
  // Find the closest point on target_ray to any point on mouse_ray.
  //
  // Math taken from http://paulbourke.net/geometry/lineline3d/
  // line P1->P2 is target_ray
  // line P3->P4 is mouse_ray

  Ogre::Vector3 v13 = target_ray.getOrigin() - mouse_ray.getOrigin();
  Ogre::Vector3 v43 = mouse_ray.getDirection();
  Ogre::Vector3 v21 = target_ray.getDirection();
  double d1343 = v13.dotProduct(v43);
  double d4321 = v43.dotProduct(v21);
  double d1321 = v13.dotProduct(v21);
  double d4343 = v43.dotProduct(v43);
  double d2121 = v21.dotProduct(v21);

  double denom = d2121 * d4343 - d4321 * d4321;
  if (fabs(denom) <= Ogre::Matrix3::EPSILON) {
    return false;
  }
  double numer = d1343 * d4321 - d1321 * d4343;

  double mua = numer / denom;
  closest_point = target_ray.getPoint(mua);
  return true;
}

void InteractiveMarkerControl::moveAxis(
  const Ogre::Ray & mouse_ray, const rviz_common::ViewportMouseEvent & event)
{
  (void)mouse_ray;

  // compute control-axis ray based on grab_point_, etc.
  Ogre::Ray control_ray;
  control_ray.setOrigin(grab_point_in_reference_frame_);
  control_ray.setDirection(control_frame_node_->getOrientation() * control_orientation_.xAxis());

  // project control-axis ray onto screen.
  Ogre::Vector2 control_ray_screen_start, control_ray_screen_end;
  worldToScreen(
    control_ray.getOrigin(),
    rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow()),
    control_ray_screen_start);
  worldToScreen(
    control_ray.getPoint(1),
    rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow()),
    control_ray_screen_end);

  Ogre::Vector2 mouse_point(event.x, event.y);

  // Find closest point on projected ray to mouse point
  // Math: if P is the start of the ray, v is the direction vector of
  //       the ray (not normalized), and X is the test point, then the
  //       closest point on the line to X is given by:
  //
  //               (X-P).v
  //       P + v * -------
  //                 v.v
  //       where "." is the dot product.
  Ogre::Vector2 control_ray_screen_dir = control_ray_screen_end - control_ray_screen_start;
  double denominator = control_ray_screen_dir.dotProduct(control_ray_screen_dir);
  // If the control ray is not straight in line with the view.
  if (fabs(denominator) > Ogre::Matrix3::EPSILON) {
    double factor =
      (mouse_point - control_ray_screen_start).dotProduct(control_ray_screen_dir) / denominator;

    Ogre::Vector2 closest_screen_point = control_ray_screen_start +
      control_ray_screen_dir * factor;

    // make a new "mouse ray" for the point on the projected ray
    Ogre::Ray new_mouse_ray = getMouseRayInReferenceFrame(
      event, closest_screen_point.x, closest_screen_point.y);

    // find closest point on control-axis ray to new mouse ray (should intersect actually)
    Ogre::Vector3 closest_point;
    if (findClosestPoint(control_ray, new_mouse_ray, closest_point)) {
      // set position of parent to closest_point - grab_point_ + parent_position_at_mouse_down_.
      parent_->setPose(
        closest_point - grab_point_in_reference_frame_ + parent_position_at_mouse_down_,
        parent_->getOrientation(),
        name_);
    }
  }
}

void InteractiveMarkerControl::moveAxis(const Ogre::Vector3 & cursor_position_in_reference_frame)
{
  Ogre::Vector3 control_unit_direction = control_frame_node_->getOrientation() *
    control_orientation_.xAxis();
  Ogre::Vector3 displacement_along_axis =
    (cursor_position_in_reference_frame - grab_point_in_reference_frame_).dotProduct(
    control_unit_direction) * control_unit_direction;

  // set position of parent to closest_point - grab_point_ + parent_position_at_mouse_down_.
  parent_->setPose(
    parent_position_at_mouse_down_ + displacement_along_axis, parent_->getOrientation(), name_);
}

void InteractiveMarkerControl::moveRotate(Ogre::Ray & mouse_ray)
{
  if (orientation_mode_ == visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING &&
    drag_viewport_)
  {
    updateControlOrientationForViewFacing(drag_viewport_);
  }

  Ogre::Vector3 new_drag_rel_ref;
  Ogre::Vector2 intersection_2d;
  float ray_t;

  // get rotation axis rel ref (constant for entire drag)
  //  - rotation_axis_

  // get current rotation center rel ref
  //  - compute rotation center rel control frame at mouse-down (constant for entire drag)
  //  - current rotation center rel ref = current control frame * above
  Ogre::Matrix4 control_rel_ref;
  control_rel_ref.makeTransform(
    control_frame_node_->getPosition(),
    Ogre::Vector3::UNIT_SCALE,
    control_frame_node_->getOrientation());
  Ogre::Vector3 rotation_center = control_rel_ref * rotation_center_rel_control_;

  // get previous 3D drag point rel ref
  //  - compute grab point rel control frame at mouse-down (constant for entire drag)
  //  - prev_drag_rel_ref = current control frame + above
  Ogre::Vector3 prev_drag_rel_ref = control_rel_ref * grab_point_rel_control_;

  // get new 3D drag point rel ref (this is "intersection_3d" in rotate().)
  //  - intersectSomeYzPlane( mouse_ray, rotation_center, control_frame_orientation )
  if (intersectSomeYzPlane(
      mouse_ray,
      rotation_center,
      control_frame_node_->getOrientation(),
      new_drag_rel_ref,
      intersection_2d,
      ray_t))
  {
    // compute rotation angle from old drag point to new.
    //  - prev_rel_center = prev_drag_rel_ref - rotation_center
    //  - new_rel_center = new_drag_rel_ref - rotation_center
    //  - rotation_change = prev_rel_center.getRotationTo( new_rel_center, rotation_axis )
    //  - get Radians of angle change
    //  - rotation_ += angle_change
    //  - parent_->rotate(rotation_change)
    Ogre::Vector3 prev_rel_center = prev_drag_rel_ref - rotation_center;
    Ogre::Vector3 new_rel_center = new_drag_rel_ref - rotation_center;
    if (new_rel_center.length() > Ogre::Matrix3::EPSILON) {
      Ogre::Quaternion rotation_change = prev_rel_center.getRotationTo(
        new_rel_center, rotation_axis_);
      Ogre::Radian rot;
      Ogre::Vector3 axis;
      rotation_change.ToAngleAxis(rot, axis);
      // Quaternion::ToAngleAxis() always gives a positive angle.  The
      // axis it emits (in this case) will either be equal to
      // rotation_axis or will be the negative of it.  Doing the
      // dot-product then gives either 1.0 or -1.0, which is just what
      // we need to get the sign for our angle.
      Ogre::Radian angle_change = rot * axis.dotProduct(rotation_axis_);
      rotation_ += angle_change;
      parent_->rotate(rotation_change, name_);

      // compute translation from rotated old drag point to new drag point.
      //  - pos_change = new_rel_center * (1.0 - prev_rel_center.length() / new_rel_center.length())
      //  - parent_->translate(pos_change)
      parent_->translate(
        new_rel_center * (1.0 - prev_rel_center.length() / new_rel_center.length()), name_);
    }
  }
}

void InteractiveMarkerControl::moveRotate(
  const Ogre::Vector3 & cursor_position_in_reference_frame, bool lock_axis)
{
  if (orientation_mode_ == visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING &&
    drag_viewport_)
  {
    updateControlOrientationForViewFacing(drag_viewport_);
  }

  // get rotation axis rel ref (constant for entire drag)
  //  - rotation_axis_

  // get current rotation center rel ref
  //  - compute rotation center rel control frame at mouse-down (constant for entire drag)
  //  - current rotation center rel ref = current control frame * above
  Ogre::Matrix4 control_rel_ref;
  control_rel_ref.makeTransform(
    control_frame_node_->getPosition(),
    Ogre::Vector3::UNIT_SCALE,
    control_frame_node_->getOrientation());
  Ogre::Vector3 rotation_center = control_rel_ref * rotation_center_rel_control_;

  // get previous 3D drag point rel ref
  //  - compute grab point rel control frame at mouse-down (constant for entire drag)
  //  - prev_drag_rel_ref = current control frame + above
  Ogre::Vector3 prev_drag_rel_ref = control_rel_ref * grab_point_rel_control_;


  Ogre::Vector3 new_drag_rel_ref = cursor_position_in_reference_frame;
  if (lock_axis) {
    Ogre::Vector3 plane_normal = control_frame_node_->getOrientation() *
      control_orientation_.xAxis();
    Ogre::Vector3 perpendicular_offset =
      (new_drag_rel_ref - prev_drag_rel_ref).dotProduct(plane_normal) * plane_normal;
    new_drag_rel_ref -= perpendicular_offset;
  }

  // compute rotation angle from old drag point to new.
  //  - prev_rel_center = prev_drag_rel_ref - rotation_center
  //  - new_rel_center = new_drag_rel_ref - rotation_center
  //  - rotation_change = prev_rel_center.getRotationTo( new_rel_center, rotation_axis )
  //  - get Radians of angle change
  //  - rotation_ += angle_change
  //  - parent_->rotate(rotation_change)
  Ogre::Vector3 prev_rel_center = prev_drag_rel_ref - rotation_center;
  Ogre::Vector3 new_rel_center = new_drag_rel_ref - rotation_center;
  if (new_rel_center.length() > Ogre::Matrix3::EPSILON) {
    Ogre::Quaternion rotation_change = prev_rel_center.getRotationTo(
      new_rel_center, rotation_axis_);
    Ogre::Radian rot;
    Ogre::Vector3 axis;
    rotation_change.ToAngleAxis(rot, axis);
    // Quaternion::ToAngleAxis() always gives a positive angle.  The
    // axis it emits (in this case) will either be equal to
    // rotation_axis or will be the negative of it.  Doing the
    // dot-product then gives either 1.0 or -1.0, which is just what
    // we need to get the sign for our angle.
    Ogre::Radian angle_change = rot * axis.dotProduct(rotation_axis_);
    rotation_ += angle_change;
    parent_->rotate(rotation_change, name_);

    // compute translation from rotated old drag point to new drag point.
    //  - pos_change = new_rel_center * (1.0 - prev_rel_center.length() / new_rel_center.length())
    //  - parent_->translate(pos_change)
    parent_->translate(
      new_rel_center * (1.0 - prev_rel_center.length() / new_rel_center.length()), name_);
  }
}

void InteractiveMarkerControl::move3D(
  const Ogre::Vector3 & cursor_position_in_reference_frame,
  const Ogre::Quaternion & cursor_orientation_in_reference_frame)
{
  if (orientation_mode_ == visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING &&
    drag_viewport_)
  {
    updateControlOrientationForViewFacing(drag_viewport_);
  }

  Ogre::Vector3 world_to_cursor_in_world_frame = reference_node_->convertLocalToWorldPosition(
    cursor_position_in_reference_frame);
  Ogre::Quaternion rotation_world_to_cursor = reference_node_->convertLocalToWorldOrientation(
    cursor_orientation_in_reference_frame);

  Ogre::Vector3 world_to_cursor_in_cursor_frame = rotation_world_to_cursor.Inverse() *
    world_to_cursor_in_world_frame;
  Ogre::Vector3 world_to_marker_in_world_frame = rotation_world_to_cursor *
    (world_to_cursor_in_cursor_frame - parent_to_cursor_in_cursor_frame_at_grab_);
  Ogre::Vector3 marker_position_in_reference_frame = reference_node_->convertWorldToLocalPosition(
    world_to_marker_in_world_frame);

  parent_->setPose(marker_position_in_reference_frame, parent_->getOrientation(), name_);
}

void InteractiveMarkerControl::rotate3D(
  const Ogre::Vector3 & cursor_position_in_reference_frame,
  const Ogre::Quaternion & cursor_orientation_in_reference_frame)
{
  (void)cursor_position_in_reference_frame;
  if (orientation_mode_ == visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING &&
    drag_viewport_)
  {
    updateControlOrientationForViewFacing(drag_viewport_);
  }

  Ogre::Quaternion rotation_world_to_cursor = reference_node_->convertLocalToWorldOrientation(
    cursor_orientation_in_reference_frame);

  Ogre::Quaternion marker_orientation_in_reference_frame =
    reference_node_->convertWorldToLocalOrientation(
    rotation_world_to_cursor * rotation_cursor_to_parent_at_grab_);

  parent_->setPose(parent_->getPosition(), marker_orientation_in_reference_frame, name_);
}


void InteractiveMarkerControl::moveRotate3D(
  const Ogre::Vector3 & cursor_position_in_reference_frame,
  const Ogre::Quaternion & cursor_orientation_in_reference_frame)
{
  if (orientation_mode_ == visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING &&
    drag_viewport_)
  {
    updateControlOrientationForViewFacing(drag_viewport_);
  }

  Ogre::Vector3 world_to_cursor_in_world_frame = reference_node_->convertLocalToWorldPosition(
    cursor_position_in_reference_frame);
  Ogre::Quaternion rotation_world_to_cursor = reference_node_->convertLocalToWorldOrientation(
    cursor_orientation_in_reference_frame);

  Ogre::Vector3 world_to_cursor_in_cursor_frame = rotation_world_to_cursor.Inverse() *
    world_to_cursor_in_world_frame;
  Ogre::Vector3 world_to_marker_in_world_frame = rotation_world_to_cursor *
    (world_to_cursor_in_cursor_frame - parent_to_cursor_in_cursor_frame_at_grab_);
  Ogre::Vector3 marker_position_in_reference_frame = reference_node_->convertWorldToLocalPosition(
    world_to_marker_in_world_frame);
  Ogre::Quaternion marker_orientation_in_reference_frame =
    reference_node_->convertWorldToLocalOrientation(
    rotation_world_to_cursor * rotation_cursor_to_parent_at_grab_);

  parent_->setPose(
    marker_position_in_reference_frame, marker_orientation_in_reference_frame, name_);
}

void InteractiveMarkerControl::setHighlight(const ControlHighlight & hl)
{
  if (hl == NO_HIGHLIGHT) {
    setHighlight(kNoHighlightValue);
  }
  if (hl == HOVER_HIGHLIGHT) {
    setHighlight(kHoverHighlightValue);
  }
  if (hl == ACTIVE_HIGHLIGHT) {
    setHighlight(kActiveHightlightValue);
  }
}

void InteractiveMarkerControl::setHighlight(float a)
{
  std::set<Ogre::Pass *>::iterator it;
  for (auto highlight : highlight_passes_) {
    highlight->setAmbient(a, a, a);
  }

  for (auto & marker : points_markers_) {
    marker->setHighlightColor(a, a, a);
  }
}

void InteractiveMarkerControl::recordDraggingInPlaceEvent(rviz_common::ViewportMouseEvent & event)
{
  dragging_in_place_event_.reset(new rviz_common::ViewportMouseEvent(event));
  dragging_in_place_event_->type = QEvent::MouseMove;
}

void InteractiveMarkerControl::stopDragging(bool force)
{
  // aleeper: Why is this check here?
  // What happens when this mouse_dragging_ check isn't done at all?
  // Or as an alternative to this minor API change, we could just manually set mouse_dragging_
  // to true before calling this function from the 3D cursor code.
  // BUT that would be a hack...
  if (mouse_dragging_ || force) {
    line_->setVisible(false);
    mouse_dragging_ = false;
    drag_viewport_ = nullptr;
    parent_->stopDragging();
  }
}

// Almost a wholesale copy of the mouse event code... can these be combined?
void InteractiveMarkerControl::handle3DCursorEvent(
  rviz_common::ViewportMouseEvent event,
  const Ogre::Vector3 & cursor_3D_pos,
  const Ogre::Quaternion & cursor_3D_orientation)
{
  // change dragging state
  switch (interaction_mode_) {
    case visualization_msgs::msg::InteractiveMarkerControl::BUTTON:
      if (event.leftUp()) {
        Ogre::Vector3 point_rel_world = cursor_3D_pos;
        bool got_3D_point = true;

        visualization_msgs::msg::InteractiveMarkerFeedback feedback;
        feedback.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK;
        feedback.control_name = name_;
        feedback.marker_name = parent_->getName();
        parent_->publishFeedback(feedback, got_3D_point, point_rel_world);
      }
      break;

    case visualization_msgs::msg::InteractiveMarkerControl::MENU:
      if (event.leftUp()) {
        // Save the 3D mouse point to send with the menu feedback, if any.
        Ogre::Vector3 three_d_point = cursor_3D_pos;
        bool valid_point = true;
        Ogre::Vector2 mouse_pos = rviz_rendering::project3DPointToViewportXY(
          rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow()),
          three_d_point);
        QCursor::setPos(event.panel->mapToGlobal(QPoint(mouse_pos.x, mouse_pos.y)));
        parent_->showMenu(event, name_, three_d_point, valid_point);
      }
      break;

    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS:
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE:
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE:
    case visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS:
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D:
    case visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D:
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D:
      if (event.leftDown()) {
        parent_->startDragging();
        drag_viewport_ = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(
          event.panel->getRenderWindow());

        grab_point_in_reference_frame_ = reference_node_->convertWorldToLocalPosition(
          cursor_3D_pos);
        grab_orientation_in_reference_frame_ = reference_node_->convertWorldToLocalOrientation(
          cursor_3D_orientation);

        parent_to_cursor_in_cursor_frame_at_grab_ = cursor_3D_orientation.Inverse() *
          (cursor_3D_pos - reference_node_->convertLocalToWorldPosition(parent_->getPosition()));
        rotation_cursor_to_parent_at_grab_ = cursor_3D_orientation.Inverse() *
          reference_node_->convertLocalToWorldOrientation(parent_->getOrientation());

        parent_position_at_mouse_down_ = parent_->getPosition();
        parent_orientation_at_mouse_down_ = parent_->getOrientation();

        if (orientation_mode_ == visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING &&
          drag_viewport_)
        {
          updateControlOrientationForViewFacing(drag_viewport_);
        }
        control_frame_orientation_at_mouse_down_ = control_frame_node_->getOrientation();
        rotation_at_mouse_down_ = rotation_;

        rotation_axis_ = control_frame_node_->getOrientation() * control_orientation_.xAxis();

        // Find rotation_center = 3D point closest to grab_point_ which is
        // on the rotation axis, relative to the reference frame.
        Ogre::Vector3 rotation_center_rel_ref = closestPointOnLineToPoint(
          parent_->getPosition(), rotation_axis_, grab_point_in_reference_frame_);
        Ogre::Matrix4 reference_rel_control_frame;
        reference_rel_control_frame.makeInverseTransform(
          control_frame_node_->getPosition(),
          Ogre::Vector3::UNIT_SCALE,
          control_frame_node_->getOrientation());
        rotation_center_rel_control_ = reference_rel_control_frame * rotation_center_rel_ref;
        grab_point_rel_control_ = reference_rel_control_frame * grab_point_in_reference_frame_;
      }
      if (event.leftUp()) {
        // Alternatively, could just manually set mouse_dragging_ to true.
        stopDragging(true);
      }
      break;
    default:
      break;
  }

  if (event.leftDown()) {
    setHighlight(kActiveHightlightValue);
  } else if (event.leftUp()) {
    setHighlight(kHoverHighlightValue);
  }

  if (!parent_->handle3DCursorEvent(event, cursor_3D_pos, cursor_3D_orientation, name_)) {
    if (event.type == QEvent::MouseMove && event.left()) {
      Ogre::Vector3 cursor_position_in_reference_frame =
        reference_node_->convertWorldToLocalPosition(cursor_3D_pos);
      Ogre::Quaternion cursor_orientation_in_reference_frame =
        reference_node_->convertWorldToLocalOrientation(cursor_3D_orientation);

      switch (interaction_mode_) {
        case visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS:
          rotate(cursor_position_in_reference_frame);
          break;

        case visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS:
          moveAxis(cursor_position_in_reference_frame);
          break;

        case visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE:
          movePlane(cursor_position_in_reference_frame);
          break;

        case visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE:
          moveRotate(cursor_position_in_reference_frame, true);
          break;

        case visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D:
          move3D(cursor_position_in_reference_frame, cursor_orientation_in_reference_frame);
          break;

        case visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D:
          rotate3D(cursor_position_in_reference_frame, cursor_orientation_in_reference_frame);
          break;

        case visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D:
          moveRotate3D(cursor_position_in_reference_frame, cursor_orientation_in_reference_frame);
          break;
        default:
          break;
      }
    }
  }
}

void InteractiveMarkerControl::handleMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  // * check if this is just a receive/lost focus event
  // * try to hand over the mouse event to the parent interactive marker
  // * otherwise, execute mouse move handling

  // handle receive/lose focus
  if (event.type == QEvent::FocusIn) {
    has_focus_ = true;
    std::set<Ogre::Pass *>::iterator it;
    setHighlight(kHoverHighlightValue);
    context_->setStatus(status_msg_);
  } else if (event.type == QEvent::FocusOut) {
    stopDragging();
    has_focus_ = false;
    setHighlight(0.0);
    return;
  }

  mouse_down_ = event.left() || event.middle() || event.right();

  // change dragging state
  switch (interaction_mode_) {
    case visualization_msgs::msg::InteractiveMarkerControl::BUTTON:
      if (event.leftUp()) {
        Ogre::Vector3 point_rel_world;
        bool got_3D_point = context_->getViewPicker()->get3DPoint(
          event.panel, event.x, event.y, point_rel_world);

        visualization_msgs::msg::InteractiveMarkerFeedback feedback;
        feedback.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK;
        feedback.control_name = name_;
        feedback.marker_name = parent_->getName();
        parent_->publishFeedback(feedback, got_3D_point, point_rel_world);
      }
      break;

    case visualization_msgs::msg::InteractiveMarkerControl::MENU:
      if (event.leftUp()) {
        Ogre::Vector3 point_rel_world;
        bool got_3D_point = context_->getViewPicker()->get3DPoint(
          event.panel, event.x, event.y, point_rel_world);
        parent_->showMenu(event, name_, point_rel_world, got_3D_point);
      }
      break;

    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS:
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE:
    case visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS:
      if (event.leftDown()) {
        beginMouseMovement(
          event,
          show_visual_aids_ &&
          orientation_mode_ != visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING);
      }
      break;

    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE:
      if (event.leftDown()) {
        beginMouseMovement(event, false);
      }
      break;

    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D:
    case visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D:
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D:
      if (event.leftDown()) {
        beginMouseMovement(event, false);
      } else {
        // Can't satisfy cpplint and uncrustify: https://github.com/ament/ament_lint/issues/158
        if (event.left() &&
          ((modifiers_at_drag_begin_ ^ event.modifiers) &
          (Qt::ShiftModifier | Qt::ControlModifier)))
        {
          // modifier buttons changed.  Restart the drag.
          beginRelativeMouseMotion(event);
        }
      }
      break;

    default:
      break;
  }

  if (!parent_->handleMouseEvent(event, name_)) {
    if (event.type == QEvent::MouseMove && event.left() && mouse_dragging_) {
      recordDraggingInPlaceEvent(event);
      handleMouseMovement(event);
    } else if (event.type == QEvent::Wheel && event.left() && mouse_dragging_) {
      handleMouseWheelMovement(event);
    }
  }

  if (event.leftDown()) {
    setHighlight(kActiveHightlightValue);
  } else if (event.leftUp()) {
    setHighlight(kHoverHighlightValue);
    stopDragging();
  }
}

void InteractiveMarkerControl::beginMouseMovement(
  rviz_common::ViewportMouseEvent & event, bool line_visible)
{
  line_->setVisible(line_visible);

  parent_->startDragging();
  mouse_dragging_ = true;
  drag_viewport_ = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(
    event.panel->getRenderWindow());

  recordDraggingInPlaceEvent(event);
  Ogre::Vector3 grab_point_in_world_frame;
  if (!context_->getViewPicker()->get3DPoint(
      event.panel, event.x, event.y, grab_point_in_world_frame))
  {
    // If we couldn't get a 3D point for the grab, just use the
    // current relative position of the control frame.
    grab_point_in_reference_frame_ = control_frame_node_->getPosition();
  } else {
    // If we could get a 3D point for the grab, convert it from
    // the world frame to the reference frame (in case those are different).
    grab_point_in_reference_frame_ = reference_node_->convertWorldToLocalPosition(
      grab_point_in_world_frame);
  }
  parent_position_at_mouse_down_ = parent_->getPosition();
  parent_orientation_at_mouse_down_ = parent_->getOrientation();

  QPoint absolute_mouse = QCursor::pos();
  mouse_relative_to_absolute_x_ = absolute_mouse.x() - event.x;
  mouse_relative_to_absolute_y_ = absolute_mouse.y() - event.y;
  beginRelativeMouseMotion(event);

  if (orientation_mode_ == visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING &&
    drag_viewport_)
  {
    updateControlOrientationForViewFacing(drag_viewport_);
  }
  control_frame_orientation_at_mouse_down_ = control_frame_node_->getOrientation();
  rotation_at_mouse_down_ = rotation_;

  rotation_axis_ = control_frame_node_->getOrientation() * control_orientation_.xAxis();

  // Find rotation_center = 3D point closest to grab_point_ which is
  // on the rotation axis, relative to the reference frame.
  Ogre::Vector3 rotation_center_rel_ref = closestPointOnLineToPoint(
    parent_->getPosition(), rotation_axis_, grab_point_in_reference_frame_);
  Ogre::Matrix4 reference_rel_control_frame;
  reference_rel_control_frame.makeInverseTransform(
    control_frame_node_->getPosition(),
    Ogre::Vector3::UNIT_SCALE,
    control_frame_node_->getOrientation());
  rotation_center_rel_control_ = reference_rel_control_frame * rotation_center_rel_ref;
  grab_point_rel_control_ = reference_rel_control_frame * grab_point_in_reference_frame_;


  // find mouse_z_scale_
  static const double DEFAULT_MOUSE_Z_SCALE = 0.001;  // default to 1mm per pixel
  mouse_z_scale_ = DEFAULT_MOUSE_Z_SCALE;

  Ogre::Ray mouse_ray = getMouseRayInReferenceFrame(event, event.x, event.y);
  Ogre::Ray mouse_ray_10 = getMouseRayInReferenceFrame(event, event.x, event.y + 10);

  Ogre::Vector3 intersection_3d;
  Ogre::Vector3 intersection_3d_10;
  Ogre::Vector2 intersection_2d;
  float ray_t;

  if (intersectSomeYzPlane(
      mouse_ray,
      grab_point_in_reference_frame_,
      control_frame_node_->getOrientation(),
      intersection_3d,
      intersection_2d,
      ray_t))
  {
    if (intersectSomeYzPlane(
        mouse_ray_10,
        grab_point_in_reference_frame_,
        control_frame_node_->getOrientation(),
        intersection_3d_10,
        intersection_2d,
        ray_t))
    {
      mouse_z_scale_ = (intersection_3d_10 - intersection_3d).length() / 10.0;
      if (mouse_z_scale_ < std::numeric_limits<float>::min() * 100.0) {
        mouse_z_scale_ = DEFAULT_MOUSE_Z_SCALE;
      }
    }
  }
}

void InteractiveMarkerControl::handleMouseMovement(rviz_common::ViewportMouseEvent & event)
{
  Ogre::Ray mouse_ray = getMouseRayInReferenceFrame(event, event.x, event.y);

  bool do_rotation = false;
  switch (interaction_mode_) {
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS:
      moveAxis(mouse_ray, event);
      break;

    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE:
      movePlane(mouse_ray);
      break;

    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE:
      moveRotate(mouse_ray);
      break;

    case visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS:
      rotate(mouse_ray);
      break;

    case visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D:
      do_rotation = true;
    // fall through
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D:
      if (event.control()) {
        do_rotation = true;
      }
    // fall through
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D:
      if (do_rotation) {
        if (event.shift()) {
          rotateZRelative(event);
        } else {
          rotateXYRelative(event);
        }
      } else {
        if (event.shift()) {
          moveZAxisRelative(event);
        } else {
          moveViewPlane(mouse_ray, event);
        }
      }
      break;

    default:
      break;
  }
}

void InteractiveMarkerControl::handleMouseWheelMovement(rviz_common::ViewportMouseEvent & event)
{
  switch (interaction_mode_) {
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D:
    case visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D:
      moveZAxisWheel(event);
      break;

    default:
      break;
  }
}

bool InteractiveMarkerControl::intersectYzPlane(
  const Ogre::Ray & mouse_ray,
  Ogre::Vector3 & intersection_3d,
  Ogre::Vector2 & intersection_2d,
  float & ray_t)
{
  return intersectSomeYzPlane(
    mouse_ray,
    control_frame_node_->getPosition(),
    control_frame_node_->getOrientation(),
    intersection_3d,
    intersection_2d,
    ray_t);
}

bool InteractiveMarkerControl::intersectSomeYzPlane(
  const Ogre::Ray & mouse_ray,
  const Ogre::Vector3 & point_on_plane,
  const Ogre::Quaternion & plane_orientation,
  Ogre::Vector3 & intersection_3d,
  Ogre::Vector2 & intersection_2d,
  float & ray_t)
{
  Ogre::Vector3 normal = plane_orientation * control_orientation_.xAxis();
  Ogre::Vector3 axis_1 = plane_orientation * control_orientation_.yAxis();
  Ogre::Vector3 axis_2 = plane_orientation * control_orientation_.zAxis();

  Ogre::Plane plane(normal, point_on_plane);

  Ogre::Vector2 origin_2d(point_on_plane.dotProduct(axis_1), point_on_plane.dotProduct(axis_2));

  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(plane);
  if (intersection.first) {
    intersection_3d = mouse_ray.getPoint(intersection.second);
    intersection_2d = Ogre::Vector2(
      intersection_3d.dotProduct(axis_1), intersection_3d.dotProduct(axis_2));
    intersection_2d -= origin_2d;

    ray_t = intersection.second;
    return true;
  }

  ray_t = 0;
  return false;
}

void InteractiveMarkerControl::addHighlightPass(markers::S_MaterialPtr materials)
{
  for (auto material : materials) {
    Ogre::Pass * original_pass = material->getTechnique(0)->getPass(0);
    Ogre::Pass * pass = material->getTechnique(0)->createPass();

    pass->setSceneBlending(Ogre::SBT_ADD);
    pass->setDepthWriteEnabled(false);
    pass->setDepthCheckEnabled(true);
    pass->setLightingEnabled(true);
    pass->setAmbient(0, 0, 0);
    pass->setDiffuse(0, 0, 0, 0);
    pass->setSpecular(0, 0, 0, 0);
    pass->setCullingMode(original_pass->getCullingMode());

    highlight_passes_.insert(pass);
  }
}

}  // namespace displays
}  // namespace rviz_default_plugins
