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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__INTERACTIVE_MARKERS__INTERACTIVE_MARKER_CONTROL_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__INTERACTIVE_MARKERS__INTERACTIVE_MARKER_CONTROL_HPP_

#ifndef Q_MOC_RUN
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <OgreRay.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#endif

#include <QCursor>

#include <visualization_msgs/msg/interactive_marker_control.hpp>

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interactive_object.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

#include "rviz_default_plugins/displays/marker/markers/marker_base.hpp"
#include "rviz_default_plugins/displays/marker/markers/points_marker.hpp"

namespace Ogre
{
class SceneNode;
}

namespace rviz_common
{
class DisplayContext;
}

namespace rviz_rendering
{
class Line;
}

namespace rviz_default_plugins
{
namespace displays
{
class InteractiveMarker;

/// A single control element of an InteractiveMarker.
class InteractiveMarkerControl
  : public Ogre::SceneManager::Listener,
  public rviz_common::InteractiveObject,
  public std::enable_shared_from_this<InteractiveMarkerControl>
{
public:
  using SharedPtr = std::shared_ptr<InteractiveMarkerControl>;

  /// Constructor.
  /**
   * Creates Ogre::SceneNodes and sets some defaults.
   * To make it look like a visualization_msgs::msg::InteractiveMarkerControl message,
   * call processMessage().
   */
  InteractiveMarkerControl(
    rviz_common::DisplayContext * context,
    Ogre::SceneNode * reference_node,
    InteractiveMarker * parent);

  virtual ~InteractiveMarkerControl();

  /// Set up or update the contents of this control to match the specification in the message.
  void processMessage(const visualization_msgs::msg::InteractiveMarkerControl & message);

  /// Called when interactive mode is globally switched on/off
  virtual void enableInteraction(bool enable);

  /// Receives all mouse events while the handler has focus
  virtual void handleMouseEvent(rviz_common::ViewportMouseEvent & event);

  /// This is the main entry-point for interaction using a 3D cursor.
  /**
   * The rviz_common::ViewportMouseEvent struct is used to "fake" a mouse event.
   * An event must have the panel, viewport, and type members filled in.
   * The acting_button and buttons_down members can be specified as well, if appropriate.
   * All other fields are currently ignored.
   *
   * A sample construction of a "right-button mouse-up" event:
   *
   * ```cpp
   * rviz_common::ViewportMouseEvent event;
   * event.panel = context_->getViewManager()->getRenderPanel();
   * event.viewport = context_->getViewManager()->getRenderPanel()->getRenderWindow()->getViewport(0);
   * event.type = QEvent::MouseButtonRelease;
   * event.acting_button = Qt::RightButton;
   * event.buttons_down = Qt::NoButton;
   * ```
   *
   * For more examples, see the implementation in the interaction_cursor_rviz package.
   *
   * \param event A struct holding certain event data (see description above).
   * \param cursor_pos The world-relative position of the 3D cursor.
   * \param cursor_orientation The world-relative orientation of the 3D cursor.
   */
  virtual void handle3DCursorEvent(
    rviz_common::ViewportMouseEvent event,
    const Ogre::Vector3 & cursor_pos,
    const Ogre::Quaternion & cursor_orientation);

  /// Update the pose of the interactive marker being controlled relative to the reference frame.
  /**
   * Each InteractiveMarkerControl maintains its pose relative to the reference frame
   * independently, so when the parent InteractiveMarker moves, it calls this function on all its
   * child controls.
   */
  void interactiveMarkerPoseChanged(
    Ogre::Vector3 int_marker_position,
    Ogre::Quaternion int_marker_orientation);

  inline bool isInteractive()
  {
    return interaction_mode_ != visualization_msgs::msg::InteractiveMarkerControl::NONE;
  }

  /// Called every frame by parent's update() function.
  void update();

  void setVisible(bool visible);

  bool getVisible();

  /// Highlight types.
  enum ControlHighlight
  {
    NO_HIGHLIGHT = 0,
    HOVER_HIGHLIGHT = 3,
    ACTIVE_HIGHLIGHT = 5
  };

  /// Public access to highlight controls.
  void setHighlight(const ControlHighlight & hl);

  /**
   * \return Pointer to the parent InteractiveMarker.
   */
  inline InteractiveMarker * getParent()
  {
    return parent_;
  }

  /**
   * \return The name of this control.
   */
  inline const std::string & getName()
  {
    return name_;
  }

  /**
   * \return The description for this control.
   */
  inline const QString & getDescription()
  {
    return description_;
  }

  /**
   * \return The visualization_msgs::msg::InteractiveMarkerControl interaction_mode
   *   for this control.
   */
  inline int getInteractionMode()
  {
    return interaction_mode_;
  }

  /**
   * \return The visualization_msgs::msg::InteractiveMarkerControl orientation_mode
   *   for this control.
   */
  inline int getOrientationMode()
  {
    return orientation_mode_;
  }

  /**
   * \param show If true, will show some geometric helpers while dragging.
   */
  inline void setShowVisualAids(bool show)
  {
    show_visual_aids_ = show;
  }

protected:
  /// When this is called, we will face the camera.
  virtual void preFindVisibleObjects(
    Ogre::SceneManager * source,
    Ogre::SceneManager::IlluminationRenderStage irs,
    Ogre::Viewport * v);

  void updateControlOrientationForViewFacing(Ogre::Viewport * v);

  /// Calculate a mouse ray in the reference frame.
  /**
   * A mouse ray is a ray starting at the camera and pointing towards the mouse position.
   */
  Ogre::Ray getMouseRayInReferenceFrame(
    const rviz_common::ViewportMouseEvent & event, int x, int y);

  /// Begin a relative-motion drag.
  void beginRelativeMouseMotion(const rviz_common::ViewportMouseEvent & event);

  /**
   * Get the relative motion of the mouse, and put the mouse back
   * where it was when beginRelativeMouseMotion() was called.
   */
  bool getRelativeMouseMotion(const rviz_common::ViewportMouseEvent & event, int & dx, int & dy);

  /// Rotate the pose around the camera-frame XY (right/up) axes, based on relative mouse movement.
  void rotateXYRelative(const rviz_common::ViewportMouseEvent & event);

  /// Rotate the pose around the camera-frame Z (look) axis, based on relative mouse movement.
  void rotateZRelative(const rviz_common::ViewportMouseEvent & event);

  /// Move the pose along the mouse ray, based on relative mouse movement.
  void moveZAxisRelative(const rviz_common::ViewportMouseEvent & event);

  /// Move the pose along the mouse ray, based on mouse wheel movement.
  void moveZAxisWheel(const rviz_common::ViewportMouseEvent & event);

  /// Move the pose around the XY view plane (perpendicular to the camera direction).
  void moveViewPlane(Ogre::Ray & mouse_ray, const rviz_common::ViewportMouseEvent & event);

  /// Rotate the pose around the local X-axis, following the mouse movement.
  void rotate(Ogre::Ray & mouse_ray);

  /// Rotate the pose around the local X axis, following the 3D cursor movement.
  void rotate(const Ogre::Vector3 & cursor_position_in_reference_frame);

  /// Translate and rotate following the mouse movement.
  /**
   * Rotate about, and translate perpendicular to, the local X-axis.
   * mouse_ray is relative to the reference frame.
   */
  void moveRotate(Ogre::Ray & mouse_ray);

  /// Translate and rotate following the 3D cursor movement.
  /**
   * Rotate about, and translate perpendicular to, the local X-axis.
   */
  void moveRotate(
    const Ogre::Vector3 & cursor_position_in_reference_frame,
    bool lock_axis = true);

  /// Translate in the plane perpendicular to the local X-axis, following the mouse movement.
  /**
   * mouse_ray is relative to the reference frame.
   */
  void movePlane(Ogre::Ray & mouse_ray);

  /// Translate in the plane perpendicular to the local X-axis, following the 3D cursor movement.
  void movePlane(const Ogre::Vector3 & cursor_position_in_reference_frame);

  /// Translate along the local X-axis, following the mouse movement.
  /**
   * mouse_ray is relative to the reference frame.
   */
  void moveAxis(const Ogre::Ray & mouse_ray, const rviz_common::ViewportMouseEvent & event);

  /// Translate along the local X-axis, following the 3D cursor movement.
  void moveAxis(const Ogre::Vector3 & cursor_position_in_reference_frame);

  /// Translate in 3-degrees-of-freedom, following the 3D cursor translation.
  void move3D(
    const Ogre::Vector3 & cursor_position_in_reference_frame,
    const Ogre::Quaternion & cursor_orientation_in_reference_frame);

  /// Rotate in 3-degrees-of-freedom, following the 3D cursor rotation.
  void rotate3D(
    const Ogre::Vector3 & cursor_position_in_reference_frame,
    const Ogre::Quaternion & cursor_orientation_in_reference_frame);

  /// Rotate and translate in full 6-DOF, following the 3D cursor movement.
  void moveRotate3D(
    const Ogre::Vector3 & cursor_position_in_reference_frame,
    const Ogre::Quaternion & cursor_orientation_in_reference_frame);

  /// Compute intersection between mouse ray and y-z plane given in local coordinates.
  bool intersectYzPlane(
    const Ogre::Ray & mouse_ray,
    Ogre::Vector3 & intersection_3d,
    Ogre::Vector2 & intersection_2d,
    float & ray_t);

  /// Compute intersection between mouse ray and a y-z plane.
  bool intersectSomeYzPlane(
    const Ogre::Ray & mouse_ray,
    const Ogre::Vector3 & point_in_plane,
    const Ogre::Quaternion & plane_orientation,
    Ogre::Vector3 & intersection_3d,
    Ogre::Vector2 & intersection_2d,
    float & ray_t);

  /// Find the closest point on target_ray to mouse_ray.
  /**
   * \param[out] closest_point Contains result point on target_ray if rays are not effectively
   *   parallel.
   * \return false if rays are effectively parallel, true otherwise.
   */
  bool findClosestPoint(
    const Ogre::Ray & target_ray,
    const Ogre::Ray & mouse_ray,
    Ogre::Vector3 & closest_point);

  /// Project a reference position onto the viewport to find screen coordinates in pixels.
  /**
   * \param[out] screen_pos the resultant screen position, in pixels.
   */
  void worldToScreen(
    const Ogre::Vector3 & pos_rel_reference,
    const Ogre::Viewport * viewport,
    Ogre::Vector2 & screen_pos);

  /// Take all the materials, add a highlight pass and store a pointer to the pass for later use.
  void addHighlightPass(markers::S_MaterialPtr materials);

  // Set the highlight color to (a,a,a)
  void setHighlight(float a);

  /**
   * Save a copy of the latest mouse event with the event type set to
   * QEvent::MouseMove, so that update() can resend the mouse event during
   * drag actions to maintain consistent behavior.
   */
  void recordDraggingInPlaceEvent(rviz_common::ViewportMouseEvent & event);

  /// Begin a new mouse motion.
  /**
   * Called when left button is pressed to begin a drag.
   */
  void beginMouseMovement(rviz_common::ViewportMouseEvent & event, bool line_visible);

  /// Motion part of mouse event handling.
  void handleMouseMovement(rviz_common::ViewportMouseEvent & event);

  /// Mouse wheel part of mouse event handling.
  void handleMouseWheelMovement(rviz_common::ViewportMouseEvent & event);

  /// Return closest point on a line to a test point.
  Ogre::Vector3 closestPointOnLineToPoint(
    const Ogre::Vector3 & line_start,
    const Ogre::Vector3 & line_dir,
    const Ogre::Vector3 & test_point);

  /// Create marker objects from the message and add them to the internal marker arrays.
  void makeMarkers(const visualization_msgs::msg::InteractiveMarkerControl & message);

  void stopDragging(bool force = false);

  virtual inline const QCursor & getCursor() const
  {
    return cursor_;
  }

  bool mouse_dragging_;

  // TODO(jacobperron): Abstract away Ogre (and other references) if possible
  Ogre::Viewport * drag_viewport_;

  std::shared_ptr<rviz_common::ViewportMouseEvent> dragging_in_place_event_;

  rviz_common::DisplayContext * context_;

  rviz_common::interaction::CollObjectHandle coll_object_handle_;

  /**
   * Node representing the reference frame in tf, like /map, /base_link,
   * /head, etc. Same as the field in InteractiveMarker.
   */
  Ogre::SceneNode * reference_node_;

  /**
   * Represents the local frame of this control relative to reference
   * node/frame.
   * There is no intermediate InteractiveMarker node or frame, each control keeps track of its
   * pose relative to the reference frame independently.
   * In INHERIT mode, this will have an identical pose as the rest of the interactive marker,
   * otherwise its orientation might be different.
   */
  Ogre::SceneNode * control_frame_node_;

  /// This is a child of scene_node, but might be oriented differently.
  Ogre::SceneNode * markers_node_;

  int interaction_mode_;

  int orientation_mode_;

  /**
   * If in view facing mode, the markers should be view facing as well.
   * If set to false, they will follow the parent's transformations.
   */
  bool independent_marker_orientation_;

  /// Defines the axis / plane along which to transform.
  /**
   * This is not keeping track of rotations applied to the control by the user,
   * this is just a copy of the "orientation" parameter from the
   * InteractiveMarkerControl message.
   */
  Ogre::Quaternion control_orientation_;

  bool always_visible_;

  QString description_;

  std::string name_;

  std::vector<markers::MarkerBase::SharedPtr> markers_;

  InteractiveMarker * parent_;

  std::set<Ogre::Pass *> highlight_passes_;

  /**
   * PointsMarkers are rendered by special shader programs, so the
   * regular highlighting method does not work for them.
   * Keep a vector of them so we can call their setHighlightColor() function.
   */
  std::vector<markers::PointsMarker::SharedPtr> points_markers_;

  /// Stores the rotation around the x axis of the control.
  /**
   * Only relevant for fixed-orientation rotation controls.
   */
  Ogre::Radian rotation_;

  /// Stores the rotation around the x axis of the control when the mouse-down event happened.
  /**
   * Only relevant for fixed-orientation rotation controls.
   */
  Ogre::Radian rotation_at_mouse_down_;

  /**
   * The 3D position of the mouse click/cursor when the 'grab' button is
   * pressed, relative to the reference frame.
   */
  Ogre::Vector3 grab_point_in_reference_frame_;

  /**
   * The orientation of the cursor when the 'grab' button is pressed, relative
   * to the reference frame.
   */
  Ogre::Quaternion grab_orientation_in_reference_frame_;

  /**
   * Records the 3D position of the cursor relative to the parent marker,
   * expressed in the cursor frame, when the 'grab' button is pressed.
   */
  Ogre::Vector3 parent_to_cursor_in_cursor_frame_at_grab_;

  /// Records the rotation of the parent from the cursor frame when the 'grab' button is pressed.
  Ogre::Quaternion rotation_cursor_to_parent_at_grab_;

  /// The modifier state when drag begins.
  Qt::KeyboardModifiers modifiers_at_drag_begin_;

  /// X-position of the mouse when drag begins.
  int mouse_x_at_drag_begin_;

  /// Y-position of the mouse when drag begins.
  int mouse_y_at_drag_begin_;

  /// Mouse ray when drag begins.
  Ogre::Ray mouse_ray_at_drag_begin_;

  /// How far to move in Z when the mouse moves 1 pixel.
  double mouse_z_scale_;

  /// X-offset of the absolute mouse position from the relative mouse position.
  int mouse_relative_to_absolute_x_;

  /// Y-offset of the absolute mouse position from the relative mouse position.
  int mouse_relative_to_absolute_y_;

  /// Position of grab relative to parent in world coordinates.
  // Ogre::Vector3 parent_to_grab_position_; // obsolete now, but left for ABI compatibility

  /// The position of the parent when the mouse button is pressed.
  Ogre::Vector3 parent_position_at_mouse_down_;

  /// The orientation of the control_frame_node_ when the mouse button is pressed.
  Ogre::Quaternion control_frame_orientation_at_mouse_down_;

  /// The orientation of the parent when the mouse button is pressed.
  Ogre::Quaternion parent_orientation_at_mouse_down_;

  /**
   * The direction vector of the axis of rotation during a mouse
   * drag, relative to the reference frame.
   * Computed on mouse down event.
   */
  Ogre::Vector3 rotation_axis_;

  /// The center of rotation during a mouse drag, relative to the control frame.
  /**
   * Computed on mouse down event.
   */
  Ogre::Vector3 rotation_center_rel_control_;

  /// The grab point during a mouse drag, relative to the control frame.
  /**
   * Computed on mouse down event.
   */
  Ogre::Vector3 grab_point_rel_control_;

  bool has_focus_;

  bool interaction_enabled_;

  bool visible_;

  bool view_facing_;

  QCursor cursor_;

  QString status_msg_;

  bool mouse_down_;

  bool show_visual_aids_;

  std::shared_ptr<rviz_rendering::Line> line_;
};  // class InteractiveMarkerControl

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__INTERACTIVE_MARKERS__INTERACTIVE_MARKER_CONTROL_HPP_
