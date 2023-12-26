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

#ifndef RVIZ_COMMON__VIEW_CONTROLLER_HPP_
#define RVIZ_COMMON__VIEW_CONTROLLER_HPP_

#include <string>
#include <memory>

#include <OgreVector.h>

#include <QCursor>  // NOLINT: cpplint is unable to handle the include order here
#include <QMap>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <Qt>  // NOLINT: cpplint is unable to handle the include order here
#include <QVariant>  // NOLINT: cpplint is unable to handle the include order here
#include <rclcpp/service.hpp>
#include <std_srvs/srv/empty.hpp>

#include "rviz_common/properties/property.hpp"
#include "rviz_common/visibility_control.hpp"

class QKeyEvent;

namespace Ogre
{

class Camera;

}  // namespace Ogre

namespace rviz_common
{

class DisplayContext;
class RenderPanel;
class ViewportMouseEvent;

namespace properties
{

class EnumProperty;
class FloatProperty;
class BoolProperty;

}  // namespace properties

struct FocalPointStatus
{
  FocalPointStatus()
  {
    exists_ = false;
    value_ = Ogre::Vector3(0, 0, 0);
  }

  FocalPointStatus(bool has_focal_point, Ogre::Vector3 focal_point)
  {
    exists_ = has_focal_point;
    value_ = focal_point;
  }

  bool exists_;
  Ogre::Vector3 value_;
};

class RVIZ_COMMON_PUBLIC ViewController : public properties::Property
{
  Q_OBJECT

public:
  ViewController();
  ~ViewController() override;

  /// Do all setup that can't be done in the constructor.
  /**
   * Creates a camera and attaches it to the root scene node.
   *
   * Calls onInitialize() just before returning.
   */
  void initialize(DisplayContext * context);

  /// Return a formatted class_id.
  static QString formatClassId(const QString & class_id);

  /// Overridden to give a different background color and bold font if this view is active.
  QVariant getViewData(int column, int role) const override;  // from Property

  /// Overridden to make this draggable if it is not active.
  Qt::ItemFlags getViewFlags(int column) const override;  // from Property

  /// Called by RenderPanel when this view controller is about to be used.
  /**
   * There is no deactivate() because ViewControllers leaving "current" are destroyed.
   * Put any cleanup in the destructor.
   */
  virtual void activate();

  /// Called at 30Hz by ViewManager::update() while this view is active.
  /**
   * Override with code that needs to run repeatedly.
   */
  virtual void update(float dt, float ros_dt);

  /// Called when mouse events are fired.
  virtual void handleMouseEvent(ViewportMouseEvent & evt);

  /// Called by MoveTool and InteractionTool when keyboard events are passed to them.
  /**
   * The default implementation here handles the "F" (focus on object) and "Z" (zero - reset) keys.
   */
  virtual void handleKeyEvent(QKeyEvent * event, RenderPanel * panel);

  /// Convenience function which calls lookAt(Ogre::Vector3).
  void lookAt(float x, float y, float z);

  // TODO(wjwwood): look at replacing Ogre::Vector3 with QPointF, to reduce Ogre dependency.
  //                see: http://doc.qt.io/qt-5/qpointf.html
  //                Or just remove it in favor of making lookAt(x, y, z) pure-virtual.
  /// Aim the camera at the given point in space.
  /**
   * The point in space is relative to the fixed frame.
   */
  virtual void lookAt(const Ogre::Vector3 & point) = 0;

  /// Reset the view controller to some sane initial state.
  /**
   * Fore example, by looking at (0, 0, 0) from a few meters away.
   */
  virtual void reset() = 0;

  /// Setup this view controller using values from the source view controller.
  /**
   * The idea is to have as similar of a view as possible when switching view
   * controllers.
   *
   * The source view controller must return a valid Ogre::Camera from getCamera().
   *
   * This base class implementation does nothing.
   */
  virtual void mimic(ViewController * source_view);

  /// Called by ViewManager when this ViewController is being made current.
  /**
   * This gives ViewController subclasses an opportunity to implement a smooth
   * transition from a previous viewpoint to the new viewpoint.
   *
   * This base class implementation does nothing.
   *
   * \param previous_view is the previous "current" view, and will not be nullptr.
   */
  virtual void transitionFrom(ViewController * previous_view);

  /// Subclasses should call this whenever a config change is made.
  /**
   * A config change is considered any change which would change the results of
   * toString()
   */
  void emitConfigChanged();

  // TODO(wjwwood): figure out how to abstract the use of Ogre::Camera
  /// Return the camera.
  Ogre::Camera * getCamera() const;

  /// Return the class identifier which was used to create this instance.
  /**
   * The default version returns whatever was set with setClassId().
   */
  virtual QString getClassId() const;

  /// Set the class identifier used to create this instance.
  /**
   * Typically this will be set by the factory object which created it.
   */
  virtual void setClassId(const QString & class_id);

  /// Load settings from a Config object.
  void load(const Config & config) override;

  /// Save settings to a Config object.
  void save(Config config) const override;

  /// Return true if this view controller is active.
  bool isActive() const;

  /// Return a mouse cursor representing the current state.
  virtual QCursor getCursor();

  virtual FocalPointStatus getFocalPointStatus();

  void resetTime();

Q_SIGNALS:
  void configChanged();

private Q_SLOTS:
  void updateNearClipDistance();
  void updateStereoProperties();
  void updateInvertZAxis();

protected:
  /// Do subclass-specific initialization.
  /**
   * Called by ViewController::initialize after context_ and camera_ are set.
   * Default implementation does nothing.
   */
  virtual void onInitialize();

  /// Called by activate().
  /**
   * Override to implement view-specific activation.
   *
   * This base implementation does nothing.
   */
  virtual void onActivate();

  // choose a cursor from the standard set
  enum CursorType
  {
    Default,
    Rotate2D,
    Rotate3D,
    MoveXY,
    MoveZ,
    Zoom,
    Crosshair
  };

  /// Set the cursor type.
  void setCursor(CursorType cursor_type);

  // Set the cursor using a custom QCursor.
  void setCursor(QCursor cursor);

  DisplayContext * context_;
  Ogre::Camera * camera_;

  bool is_active_;

  // this cursor will be displayed when the mouse is within the
  // window controlled by this view controller
  // use SetCursor to modify.
  QCursor cursor_;

  rviz_common::properties::FloatProperty * near_clip_property_;
  rviz_common::properties::BoolProperty * stereo_enable_;
  rviz_common::properties::BoolProperty * stereo_eye_swap_;
  rviz_common::properties::FloatProperty * stereo_eye_separation_;
  rviz_common::properties::FloatProperty * stereo_focal_distance_;
  rviz_common::properties::BoolProperty * invert_z_;

  /// Set the status on the main render frame.
  void setStatus(const QString & message);

private:
  rviz_common::properties::EnumProperty * type_property_;
  QString class_id_;

  // Default cursors for the most common actions
  QMap<CursorType, QCursor> standard_cursors_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_time_srv_;

  void resetService(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>);
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__VIEW_CONTROLLER_HPP_
