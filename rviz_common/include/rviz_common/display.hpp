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

#ifndef RVIZ_COMMON__DISPLAY_HPP_
#define RVIZ_COMMON__DISPLAY_HPP_

#include <string>

#include <QIcon>  // NOLINT: cpplint is unable to handle the include order here
#include <QSet>  // NOLINT: cpplint is unable to handle the include order here

#include "rclcpp/time.hpp"

#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/visibility_control.hpp"

class QDockWidget;
class QWidget;

namespace Ogre
{
class SceneManager;
class SceneNode;
}

// Required, in combination with `qRegisterMetaType<rclcpp::Time>` in the cpp
// file, so that this type can be used with a Qt signal.
// See: http://doc.qt.io/qt-5/qmetatype.html#qRegisterMetaType-1
Q_DECLARE_METATYPE(rclcpp::Time)

namespace rviz_common
{

namespace properties
{

class StatusList;

}  // namespace properties

class DisplayContext;
class PanelDockWidget;

class RVIZ_COMMON_PUBLIC Display : public rviz_common::properties::BoolProperty
{
  Q_OBJECT

public:
  Display();
  virtual ~Display();

  /// Main initialization, called after constructor, before load() or setEnabled().
  virtual
  void
  initialize(DisplayContext * context);

  /// Return data appropriate for the given column (0 or 1) and role for this Display.
  QVariant
  getViewData(int column, int role) const override;

  /// Return item flags appropriate for the given column (0 or 1) for this Display.
  Qt::ItemFlags
  getViewFlags(int column) const override;

  /// Return the class identifier which was used to create this instance.
  /**
   * This version just returns whatever was set with setClassId().
   */
  virtual
  QString
  getClassId() const;

  /// Set the class identifier used to create this instance.
  /**
   * Typically this will be set by the factory object which created it.
   */
  virtual
  void
  setClassId(const QString & class_id);

  /// Load the settings for this display from the given Config node, which must be a map.
  /**
   * Overridden from Property::load() to load the Display's name and enabled
   * state, then call Property::load().
   *
   * load() is called after initialize().
   */
  void
  load(const Config & config) override;

  /// Write this display to the given Config node.
  /**
   * Overridden from Property::save().
   */
  void
  save(Config config) const override;

  /// Set the ROS topic to listen to for this display.
  /**
   * By default, do nothing.
   * Subclasses should override this method if they subscribe to a single
   * ROS topic.
   *
   * setTopic() is used by the "New display by topic" window; it is called
   * with a user selected topic and its type.
   *
   * \param topic The published topic to be visualized.
   * \param datatype The datatype of the topic.
   */
  virtual
  void
  setTopic(const QString & topic, const QString & datatype);

  /// Return true if this Display is enabled, false if not.
  bool
  isEnabled() const;

  /// Set the fixed frame in this display.
  void
  setFixedFrame(const QString & fixed_frame);

  /// Called periodically by the visualization manager.
  /**
   * \param wall_dt Wall-clock time, in seconds, since the last time the update list was run through.
   * \param ros_dt ROS time, in seconds, since the last time the update list was run through.
   */
  virtual
  void
  update(float wall_dt, float ros_dt);

  /// Called to tell the display to clear its state.
  virtual
  void
  reset();

  /// Show status level and text.
  /**
   * This is thread-safe.
   *
   * Every Display has a StatusList to indicate how it is doing.  The
   * StatusList has StatusProperty children indicating the status of
   * various subcomponents of the Display.  Each child of the status
   * has a level, a name, and descriptive text.  The top-level
   * StatusList has a level which is set to the worst of all the
   * children's levels.
   *
   * \param level One of StatusProperty::Ok, StatusProperty::Warn, or
   *   StatusProperty::Error.
   * \param name The name of the child entry to set.
   * \param text Description of the child's state.
   */
  virtual
  void
  setStatus(properties::StatusProperty::Level level, const QString & name, const QString & text);

  /// Show status level and text, using a std::string.
  /**
   * Convenience function which converts std::string to QString and calls
   * setStatus().
   * This is thread-safe.
   */
  void
  setStatusStd(
    properties::StatusProperty::Level level,
    const std::string & name,
    const std::string & text);

  /// Convenience: Show and log missing transform
  /**
   * Convenience function which
   * @param frame frame with missing transform to fixed_frame
   * @param additional_info additional info included in the error_message which then reads "Could
   * not transform <additional_info> from [<fixed_frame>] to [<frame>].
   */
  void
  setMissingTransformToFixedFrame(
    const std::string & frame, const std::string & additional_info = "");

  /// Convenience: Set Transform ok
  void
  setTransformOk();

  /// Delete the status entry with the given name.
  /**
   * This is thread-safe.
   */
  virtual
  void
  deleteStatus(const QString & name);

  /// Delete the status entry with the given std::string name.
  /**
   * This is thread-safe.
   */
  void
  deleteStatusStd(const std::string & name);

  /// Set the visibility bits.
  /**
   * Default is all bits ON.
   */
  void
  setVisibilityBits(uint32_t bits);

  /// Unset the visibility bits.
  void
  unsetVisibilityBits(uint32_t bits);

  /// Get the visibility bits.
  uint32_t
  getVisibilityBits();

  /// Return the Ogre::SceneNode holding all 3D scene elements shown by this Display.
  Ogre::SceneNode *
  getSceneNode() const;

  /// Associate the given @a widget with this Display.
  /**
   * Each Display can have one QWidget which is shown when the Display
   * is enabled and hidden when the Display is disabled.
   * If there is a WindowManagerInterface registered with the
   * VisualizationManager, like if you are using a VisualizationFrame,
   * this also adds widget as a pane within it (with
   * WindowManagerInterface::addPane()).
   *
   * Since there is only one slot for such a widget, this dis-associates any
   * previously associated widget.
   *
   * Call this with nullptr to disassociate the current associated widget.
   */
  void
  setAssociatedWidget(QWidget * widget);

  /// Return the current associated widget, or nullptr if there is none.
  /**
   * \see setAssociatedWidget()
   */
  QWidget *
  getAssociatedWidget() const;

  /// Return the panel containing the associated widget, or nullptr if there is none.
  /**
   * \see setAssociatedWidget()
   */
  PanelDockWidget *
  getAssociatedWidgetPanel();

  /// Set associated widget title to the name.
  void
  setName(const QString & name) final;  // Overridden from Property.

  /// Emit a time signal that other Displays can synchronize to.
  void
  emitTimeSignal(rclcpp::Time time);

  virtual
  properties::Property *
  findProperty(const QString & name);

  /// Get the latest transform to the frame and update the scene node. Return true on success.
  bool updateFrame(const std::string & frame);

  /// Get transform to the frame at the given time and update the scene node. True on success.
  bool updateFrame(const std::string & frame, rclcpp::Time time);

Q_SIGNALS:
  void
  timeSignal(rviz_common::Display * display, rclcpp::Time time);

public Q_SLOTS:
  /// Enable or disable this Display.
  /**
   * SetEnabled is called after initialize() and at the end of load(),
   * if the Display settings are being loaded from a file.
   */
  void
  setEnabled(bool enabled);

  /// Convenience function which calls context_->queueRender().
  void
  queueRender();

  /// Set the Display's icon.
  void
  setIcon(const QIcon & icon) override;

protected:
  /// Override this function to do subclass-specific initialization.
  /**
   * This is called after vis_manager_ and scene_manager_ are set, and before
   * load() or setEnabled().
   *
   * setName() may or may not have been called before this.
   */
  virtual
  void
  onInitialize();

  /// Derived classes override this to do the actual work of enabling themselves.
  virtual
  void
  onEnable();

  /// Derived classes override this to do the actual work of disabling themselves.
  virtual
  void
  onDisable();

  /// Delete all status children.
  /**
   * This is thread-safe.
   *
   * This removes all status children and updates the top-level status.
   */
  virtual
  void
  clearStatuses();

  /// Called by setFixedFrame().
  /**
   * Override to respond to changes to fixed_frame_.
   */
  virtual
  void
  fixedFrameChanged();

  /// Returns true if the display has been initialized.
  bool
  initialized() const;

  /// This DisplayContext pointer is the main connection a Display has into the rest of rviz.
  /**
   * This is how the FrameManager is accessed, the SelectionManager, etc.
   * When a Display subclass wants to signal that a new render should be done
   * right away, call context_->queueRender().
   *
   * This is set after the constructor and before onInitialize() is called.
   */
  DisplayContext * context_;

  /// A convenience variable equal to context_->getSceneManager().
  /**
   * This is set after the constructor and before onInitialize() is called.
   */
  Ogre::SceneManager * scene_manager_;

  /// The Ogre::SceneNode to hold all 3D scene elements shown by this Display.
  Ogre::SceneNode * scene_node_;

  /// A convenience variable equal to context_->getFixedFrame().
  /**
   * This is set after the constructor and before onInitialize() is
   * called.
   * Every time it is updated (via setFixedFrame()), fixedFrameChanged() is called.
   */
  QString fixed_frame_;

public Q_SLOTS:
  virtual
  void
  onEnableChanged();

private Q_SLOTS:
  void
  setStatusInternal(int level, const QString & name, const QString & text);

  void
  deleteStatusInternal(const QString & name);

  void
  clearStatusesInternal();

  void
  associatedPanelVisibilityChange(bool visible);

  void
  disable();

private:
  rviz_common::properties::StatusList * status_;
  QString class_id_;
  bool initialized_;
  uint32_t visibility_bits_;
  QWidget * associated_widget_;
  PanelDockWidget * associated_widget_panel_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__DISPLAY_HPP_
