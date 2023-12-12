/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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

#ifndef RVIZ_COMMON__VISUALIZATION_FRAME_HPP_
#define RVIZ_COMMON__VISUALIZATION_FRAME_HPP_

#include <chrono>
#include <deque>
#include <map>
#include <string>

#include <QList>  // NOLINT: cpplint is unable to handle the include order here
#include <QMainWindow>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <Qt>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/config.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/window_manager_interface.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

class QAction;
class QActionGroup;
class QApplication;
class QCloseEvent;
class QDockWidget;
class QEvent;
class QLabel;
class QSplashScreen;
class QTimer;
class QToolButton;
class QWidget;

namespace rviz_common
{

class Panel;
class PanelDockWidget;
class PanelFactory;
class RenderPanel;
class Tool;
class VisualizationManager;
class WidgetGeometryChangeDetector;

/// The main rviz window.
/**
 * VisualizationFrame is a QMainWindow, which means it has a center area and a
 * bunch of dock areas around it.
 * The central widget here is a RenderPanel, and around it (by default) are the
 * DisplaysPanel, ViewsPanel, TimePanel, SelectionPanel, and
 * ToolPropertiesPanel.
 * At the top is a toolbar with Tools like "Move Camera", "Select", etc.
 * There is also a menu bar with file/open, etc.
 */
class RVIZ_COMMON_PUBLIC VisualizationFrame : public QMainWindow, public WindowManagerInterface
{
  Q_OBJECT

public:
  explicit VisualizationFrame(
    ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget * parent = nullptr);
  ~VisualizationFrame() override;

  rviz_rendering::RenderWindow * getRenderWindow();

  /// Set the QApplication, this should be called directly after construction.
  void
  setApp(QApplication * app);

  /// Set the path to the html help file.
  /**
   * Default is a file within the rviz_common package.
   */
  void
  setHelpPath(const QString & help_path);

  /// Set the path to the "splash" image file.
  /**
   * This image is shown during initialization and loading of the config file.
   * Default is a file within the rviz_common package.
   * To prevent splash image from showing, set this to an empty string.
   */
  void
  setSplashPath(const QString & splash_path);

  /// Initialize the VisualizationFrame and create the VisualizationManager.
  /**
   * This function must be called before load(), save(), getManager(),
   * or addPanelByName(), because it creates the VisualizationManager
   * instance which those calls depend on.
   *
   * This function also calls VisualizationManager::initialize(),
   * which means it will start the update timer and generally get
   * things rolling.
   */
  void
  initialize(
    ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
    const QString & display_config_file = "");

  /// Set the display title format.
  /**
   * Sets the format of the window title.
   * Three replacement tokens are supported:
   *  - {NAMESPACE} - replace with the namespace this node is in
   *  - {CONFIG_PATH} - replace with the path (but not the filename) of the configuration file in use.
   *  - {CONFIG_FILENAME} - replace with the filename (but not the path) of the configuration file in use.
   * The default is "RViz[*]" if the default configuration file is in use,
   * or "{CONFIG_PATH}/{CONFIG_FILENAME}[*] - RViz" if a custom configuration file is in use.
   */
  void
  setDisplayTitleFormat(const QString & title_format);

  /// Return the visualization manager.
  VisualizationManager *
  getManager();

  // Overriden from WindowManagerInterface:
  QWidget *
  getParentWindow() override;

  // Overriden from WindowManagerInterface:
  PanelDockWidget *
  addPane(
    const QString & name,
    QWidget * panel,
    Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
    bool floating = true) override;

  /// Load the "general", persistent settings from a file.
  /**
   * This config file has a few things which should not be saved within
   * a display config.
   */
  void
  loadPersistentSettings();

  /// Save the "general", persistent settings to a file.
  /**
   * This config file has a few things which should not be saved within
   * a display config.
   */
  void
  savePersistentSettings();

  /// Load display settings from the given file.
  /**
   * \param path The full path of the config file to load from.
   */
  void
  loadDisplayConfig(const QString & path);

  // TODO(wjwwood): consider changing this function to raise an exception
  //                when there is a failure, rather than the getErrorMessage()
  //                mechanism, unless there is a good reason for it.
  /// Save display settings to the given file.
  /**
   * On failure, also sets error_message_ with information about the
   * problem.
   * The error message can be retrieved with getErrorMessage().
   *
   * \param path The full path of the config file to save into.
   * \return true on success, and false on failure.
   */
  bool
  saveDisplayConfig(const QString & path);

  /// Return the error message set by saveDisplayConfig().
  QString
  getErrorMessage() const;

  /// Load the properties of all subsystems from the given Config.
  /**
   * This is called by loadDisplayConfig().
   *
   * \param config Config object of 'type' Config::Map.
   */
  virtual
  void
  load(const Config & config);

  /// Save the properties of each subsystem and most editable rviz data.
  /**
   * This is called by saveDisplayConfig().
   *
   * \param config a Config object to write into.
   */
  virtual
  void
  save(Config config);

  /// Hide or show the hide-dock buttons.
  void
  setHideButtonVisibility(bool visible);

  /// Add a panel by a given name and class name.
  QDockWidget *
  addPanelByName(
    const QString & name,
    const QString & class_lookup_name,
    Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
    bool floating = true);

public Q_SLOTS:
  /// Notification that something would change in the display config if saved.
  void
  setDisplayConfigModified();

  /// Set the message displayed in the status bar.
  void
  setStatus(const QString & message) override;

  /// Set full screen mode.
  void
  setFullScreen(bool full_screen);

  /// Exit full screen mode.
  void
  exitFullScreen();

Q_SIGNALS:
  /// Emitted during file-loading and initialization to indicate progress.
  void
  statusUpdate(const QString & message);

  /// Emitted when the interface enters or leaves full screen mode.
  void
  fullScreenChange(bool hidden);

protected Q_SLOTS:
  /// Handle event to open a display config file.
  void
  onOpen();

  /// Handle event to save to the current display config file.
  void
  onSave();

  /// Handle event to save the current display config to a different file.
  void
  onSaveAs();

  /// Handle event to save a screenshot of the current rviz window.
  void
  onSaveImage();

  /// Handle QActions, often fired when panels are added or removed.
  void
  onRecentConfigSelected();

  /// Handle event to show the about dialog.
  void
  onHelpAbout();

  /// Handle event to open the new panel dialog.
  void
  openNewPanelDialog();

  /// Handle event to open the new tool dialog.
  void
  openNewToolDialog();

  /// Handle event to show the help panel.
  void
  showHelpPanel();

  /// Remove a the tool whose name is given by remove_tool_menu_action->text().
  void
  onToolbarRemoveTool(QAction * remove_tool_menu_action);

  /// Look up the Tool for this action and call VisualizationManager::setCurrentTool().
  void
  onToolbarActionTriggered(QAction * action);

  /// Add the given tool to this frame's toolbar.
  /**
   * This creates a QAction internally which listens for the Tool's
   * shortcut key.
   * When the action is triggered by the toolbar or by the shortcut key,
   * onToolbarActionTriggered() is called.
   */
  void
  addTool(Tool * tool);

  /// React to name changes of a tool, updating the name of the associated QAction
  void onToolNameChanged(const QString & name);

  /// Remove the given tool from the frame's toolbar.
  void
  removeTool(Tool * tool);

  /// Refresh the given tool in this frame's toolbar.
  /**
   * This will update the icon and the text of the corresponding QAction.
   */
  void
  refreshTool(Tool * tool);

  /// Mark the given tool as the current one.
  /**
   * This is purely a visual change in the GUI, it does not call any
   * tool functions.
   */
  void
  indicateToolIsCurrent(Tool * tool);

  /// Delete a panel widget.
  /**
   * The sender() of the signal should be a QAction whose text() is
   * the name of the panel.
   */
  void
  onDeletePanel();

  /// Indicate that loading is done.
  void
  markLoadingDone();

  /// Set the default directory in which to save screenshot images.
  void
  setImageSaveDirectory(const QString & directory);

  /// Reset the render window.
  /**
   * This will clear the loaded meshes, reset any time tracking, and possibly
   * other things.
   */
  void
  reset();

  // TODO(wjwwood): figure out if this is needed
  /// Handle event when the help dialog is closed.
  void
  onHelpDestroyed();

  /// Hide the left dock area.
  void
  hideLeftDock(bool hide);

  /// Hide the right dock area.
  void
  hideRightDock(bool hide);

  /// Handle event when the dock panel visibility changes.
  virtual
  void
  onDockPanelVisibilityChange(bool visible);

  /// Handle request to update the current frames per second (FPS).
  void
  updateFps();

protected:
  // TODO(wjwwood): figure out what the correct thing to do in ROS 2 here is
  //                are we still using ~/.ros and/or ~/.rviz, or ~/.rviz2?
  /// Initialize the default config directory, which defaults to '~/.rviz'.
  void
  initConfigs();

  /// Setup the menu bar and menus.
  void
  initMenus();

  /// Setup the toolbar and the tools in it.
  void
  initToolbars();

  /// Check for unsaved changes, prompt to save config, etc.
  /**
   * \return true if it is OK to exit, false if not.
   */
  bool
  prepareToExit();

  /// Called when the user attempts to close the window.
  void
  closeEvent(QCloseEvent * event) override;

  /// Called when the mouse cursor leaves the window.
  void
  leaveEvent(QEvent * event) override;

  /// Called when the current display config file changes.
  void
  markRecentConfig(const std::string & path);

  /// Called by markRecentConfig().
  void
  updateRecentConfigMenu();

  /// Loads custom panels from the given Config object.
  void
  loadPanels(const Config & config);

  /// Saves custom panels to the given Config object.
  void
  savePanels(Config config);

  /// Restore the window's geometry from the given Config object.
  void
  loadWindowGeometry(const Config & config);

  /// Save the window's geometry to the given Config object.
  void
  saveWindowGeometry(Config config);

  /// Set the display config file path.
  /**
   * This does not load the given file, it just sets the member
   * variable and updates the window title.
   */
  void
  setDisplayConfigFile(const std::string & path);

  /// Hide or show the given dock area based on the hide bool.
  void
  hideDockImpl(Qt::DockWidgetArea area, bool hide);

  /// Parent QApplication, set by setApp().
  QApplication * app_;

  /// Actual panel where the main 3D scene is rendered.
  RenderPanel * render_panel_;

  // TODO(wjwwood): setup this class with a PIMPL class to hide all these
  //                implementation variables, to make it easier to provide
  //                ABI compatibility in the future
  QAction * show_help_action_;

  std::string config_dir_;
  std::string persistent_settings_file_;
  std::string display_title_format_;
  std::string display_config_file_;
  std::string default_display_config_file_;
  std::string last_config_dir_;
  std::string last_image_dir_;
  std::string home_dir_;

  QMenu * file_menu_;
  QMenu * recent_configs_menu_;
  QMenu * view_menu_;
  QMenu * delete_view_menu_;
  QMenu * plugins_menu_;

  QToolBar * toolbar_;

  VisualizationManager * manager_;

  std::string package_path_;
  QString help_path_;
  QString splash_path_;

  QSplashScreen * splash_;

  typedef std::deque<std::string> D_string;
  D_string recent_configs_;

  QActionGroup * toolbar_actions_;
  std::map<QAction *, Tool *> action_to_tool_map_;
  std::map<Tool *, QAction *> tool_to_action_map_;
  bool show_choose_new_master_option_;

  QToolButton * hide_left_dock_button_;
  QToolButton * hide_right_dock_button_;

  PanelFactory * panel_factory_;

  struct PanelRecord
  {
    Panel * panel;
    PanelDockWidget * dock;
    QString name;
    QString class_id;
    QAction * delete_action;
  };
  QList<PanelRecord> custom_panels_;

  QAction * add_tool_action_;
  QMenu * remove_tool_menu_;

  bool initialized_;
  WidgetGeometryChangeDetector * geom_change_detector_;
  /// True just when loading a display config file, false all other times.
  bool loading_;
  /// Single-shot timer for calling postLoad() a short time after loadDisplayConfig() finishes.
  QTimer * post_load_timer_;

  QLabel * status_label_;
  QLabel * fps_label_;
  QStatusBar * original_status_bar_;

  int frame_count_;
  std::chrono::steady_clock::time_point last_fps_calc_time_;

  /// Error message (if any) from most recent saveDisplayConfig() call.
  QString error_message_;

  /// Indicates if the toolbar should be visible outside of fullscreen mode.
  bool toolbar_visible_;

  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__VISUALIZATION_FRAME_HPP_
