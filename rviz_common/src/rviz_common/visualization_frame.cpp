/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz_common/visualization_frame.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include <OgreRenderWindow.h>
#include <OgreMeshManager.h>
#include <OgreMaterialManager.h>

#include <QApplication>  // NOLINT cpplint cannot handle include order here
#include <QCloseEvent>  // NOLINT cpplint cannot handle include order here
#include <QDesktopServices>  // NOLINT cpplint cannot handle include order here
#include <QDir>  // NOLINT cpplint cannot handle include order here
#include <QFile>  // NOLINT cpplint cannot handle include order here
#include <QFileDialog>  // NOLINT cpplint cannot handle include order here
#include <QHBoxLayout>  // NOLINT cpplint cannot handle include order here
#include <QMenu>  // NOLINT cpplint cannot handle include order here
#include <QMenuBar>  // NOLINT cpplint cannot handle include order here
#include <QMessageBox>  // NOLINT cpplint cannot handle include order here
#include <QShortcut>  // NOLINT cpplint cannot handle include order here
#include <QSplashScreen>  // NOLINT cpplint cannot handle include order here
#include <QStatusBar>  // NOLINT cpplint cannot handle include order here
#include <QTimer>  // NOLINT cpplint cannot handle include order here
#include <QToolBar>  // NOLINT cpplint cannot handle include order here
#include <QToolButton>  // NOLINT cpplint cannot handle include order here

#include "rclcpp/clock.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rviz_common/load_resource.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/panel_dock_widget.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/yaml_config_reader.hpp"
#include "rviz_common/yaml_config_writer.hpp"
#include "rviz_rendering/render_window.hpp"

#include "./env_config.hpp"
#include "./failed_panel.hpp"
#include "./loading_dialog.hpp"
#include "./new_object_dialog.hpp"
#include "./panel_factory.hpp"
#include "./screenshot_dialog.hpp"
#include "./splash_screen.hpp"
#include "rviz_common/tool_manager.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "./widget_geometry_change_detector.hpp"

// #include "./displays_panel.hpp"
#include "./help_panel.hpp"
// #include "./interaction/selection_manager.hpp"
// #include "./selection_panel.hpp"
// #include "./time_panel.hpp"
// #include "./tool_properties_panel.hpp"
// #include "./views_panel.hpp"

#define CONFIG_EXTENSION "rviz"
#define CONFIG_EXTENSION_WILDCARD "*." CONFIG_EXTENSION
#define RECENT_CONFIG_COUNT 10

namespace rviz_common
{

VisualizationFrame::VisualizationFrame(
  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget * parent)
: QMainWindow(parent),
  app_(nullptr),
  render_panel_(nullptr),
  show_help_action_(nullptr),
  file_menu_(nullptr),
  recent_configs_menu_(nullptr),
  toolbar_(nullptr),
  manager_(nullptr),
  splash_(nullptr),
  toolbar_actions_(nullptr),
  show_choose_new_master_option_(false),
  panel_factory_(nullptr),
  add_tool_action_(nullptr),
  remove_tool_menu_(nullptr),
  initialized_(false),
  geom_change_detector_(new WidgetGeometryChangeDetector(this)),
  loading_(false),
  post_load_timer_(new QTimer(this)),
  frame_count_(0),
  toolbar_visible_(true),
  rviz_ros_node_(rviz_ros_node)
{
  setObjectName("VisualizationFrame");
  installEventFilter(geom_change_detector_);
  connect(geom_change_detector_, SIGNAL(changed()), this, SLOT(setDisplayConfigModified()));

  post_load_timer_->setSingleShot(true);
  connect(post_load_timer_, SIGNAL(timeout()), this, SLOT(markLoadingDone()));

  package_path_ = ament_index_cpp::get_package_share_directory("rviz_common");
  QDir help_path(QString::fromStdString(package_path_) + "/help/help.html");
  help_path_ = help_path.absolutePath();
  QDir splash_path(QString::fromStdString(package_path_) + "/images/splash.png");
  splash_path_ = splash_path.absolutePath();

  auto * reset_button = new QToolButton();
  reset_button->setText("Reset");
  reset_button->setContentsMargins(0, 0, 0, 0);
  statusBar()->addPermanentWidget(reset_button, 0);
  connect(reset_button, SIGNAL(clicked(bool)), this, SLOT(reset()));

  status_label_ = new QLabel("");
  statusBar()->addPermanentWidget(status_label_, 1);
  connect(this, SIGNAL(statusUpdate(const QString&)), status_label_, SLOT(setText(const QString&)));

  fps_label_ = new QLabel("");
  fps_label_->setMinimumWidth(40);
  fps_label_->setAlignment(Qt::AlignRight);
  statusBar()->addPermanentWidget(fps_label_, 0);
  original_status_bar_ = statusBar();

  setWindowTitle("RViz[*]");
}

VisualizationFrame::~VisualizationFrame()
{
  delete manager_;
  delete render_panel_;

  for (auto & custom_panel : custom_panels_) {
    delete custom_panel.dock;
  }

  delete panel_factory_;
}

rviz_rendering::RenderWindow * VisualizationFrame::getRenderWindow()
{
  return render_panel_->getRenderWindow();
}

void VisualizationFrame::setApp(QApplication * app)
{
  app_ = app;
}

void VisualizationFrame::setStatus(const QString & message)
{
  Q_EMIT statusUpdate(message);
}

void VisualizationFrame::updateFps()
{
  frame_count_++;
  auto wall_diff = std::chrono::steady_clock::now() - last_fps_calc_time_;

  if (wall_diff > std::chrono::seconds(1)) {
    using std::chrono::duration;
    float fps = frame_count_ / std::chrono::duration_cast<duration<double>>(wall_diff).count();
    frame_count_ = 0;
    last_fps_calc_time_ = std::chrono::steady_clock::now();
    if (original_status_bar_ == statusBar()) {
      fps_label_->setText(QString::number(static_cast<int>(fps)) + QString(" fps"));
    }
  }
}

void VisualizationFrame::closeEvent(QCloseEvent * event)
{
  if (prepareToExit()) {
    event->accept();
  } else {
    event->ignore();
  }
}

void VisualizationFrame::leaveEvent(QEvent * event)
{
  Q_UNUSED(event);
  setStatus("");
}

void VisualizationFrame::reset()
{
  Ogre::MeshManager::getSingleton().removeAll();
  manager_->resetTime();
}

void VisualizationFrame::setHelpPath(const QString & help_path)
{
  help_path_ = help_path;
  manager_->setHelpPath(help_path_);
}

void VisualizationFrame::setSplashPath(const QString & splash_path)
{
  splash_path_ = splash_path;
}

void VisualizationFrame::initialize(
  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
  const QString & display_config_file)
{
  initConfigs();

  loadPersistentSettings();

  if (app_) {
    QDir app_icon_path(QString::fromStdString(package_path_) + "/icons/package.png");
    QIcon app_icon(app_icon_path.absolutePath());
    app_->setWindowIcon(app_icon);
  }

  if (splash_path_ != "") {
    QPixmap splash_image(splash_path_);
    splash_ = new SplashScreen(splash_image);
    splash_->show();
    connect(this, SIGNAL(statusUpdate(const QString&)), splash_, SLOT(showMessage(const QString&)));
  }
  Q_EMIT statusUpdate("Initializing");

  // Periodically process events for the splash screen.
  // See: http://doc.qt.io/qt-5/qsplashscreen.html#details
  QCoreApplication::processEvents();

  // Periodically process events for the splash screen.
  QCoreApplication::processEvents();

  QWidget * central_widget = new QWidget(this);
  QHBoxLayout * central_layout = new QHBoxLayout;
  central_layout->setSpacing(0);
  central_layout->setMargin(0);

  render_panel_ = new RenderPanel(central_widget);

  hide_left_dock_button_ = new QToolButton();
  hide_left_dock_button_->setContentsMargins(0, 0, 0, 0);
  hide_left_dock_button_->setArrowType(Qt::LeftArrow);
  hide_left_dock_button_->setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding));
  hide_left_dock_button_->setFixedWidth(16);
  hide_left_dock_button_->setAutoRaise(true);
  hide_left_dock_button_->setCheckable(true);

  connect(hide_left_dock_button_, SIGNAL(toggled(bool)), this, SLOT(hideLeftDock(bool)));

  hide_right_dock_button_ = new QToolButton();
  hide_right_dock_button_->setContentsMargins(0, 0, 0, 0);
  hide_right_dock_button_->setArrowType(Qt::RightArrow);
  hide_right_dock_button_->setSizePolicy(
    QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding));
  hide_right_dock_button_->setFixedWidth(16);
  hide_right_dock_button_->setAutoRaise(true);
  hide_right_dock_button_->setCheckable(true);

  connect(hide_right_dock_button_, SIGNAL(toggled(bool)), this, SLOT(hideRightDock(bool)));

  central_layout->addWidget(hide_left_dock_button_, 0);
  central_layout->addWidget(render_panel_, 1);
  central_layout->addWidget(hide_right_dock_button_, 0);

  central_widget->setLayout(central_layout);

  // Periodically process events for the splash screen.
  QCoreApplication::processEvents();

  initMenus();

  // Periodically process events for the splash screen.
  QCoreApplication::processEvents();

  initToolbars();

  // Periodically process events for the splash screen.
  QCoreApplication::processEvents();

  setCentralWidget(central_widget);

  // Periodically process events for the splash screen.
  QCoreApplication::processEvents();

  // TODO(wjwwood): sort out the issue with initialization order between
  //                render_panel and VisualizationManager
  render_panel_->getRenderWindow()->initialize();

  auto clock = rviz_ros_node.lock()->get_raw_node()->get_clock();
  manager_ = new VisualizationManager(render_panel_, rviz_ros_node, this, clock);
  manager_->setHelpPath(help_path_);
  panel_factory_ = new PanelFactory(rviz_ros_node_, manager_);

  // Periodically process events for the splash screen.
  QCoreApplication::processEvents();

  render_panel_->initialize(manager_);

  // Periodically process events for the splash screen.
  QCoreApplication::processEvents();

  ToolManager * tool_man = manager_->getToolManager();

  connect(manager_, SIGNAL(configChanged()), this, SLOT(setDisplayConfigModified()));
  connect(tool_man, SIGNAL(toolAdded(Tool*)), this, SLOT(addTool(Tool*)));
  connect(tool_man, SIGNAL(toolRemoved(Tool*)), this, SLOT(removeTool(Tool*)));
  connect(tool_man, SIGNAL(toolRefreshed(Tool*)), this, SLOT(refreshTool(Tool*)));
  connect(tool_man, SIGNAL(toolChanged(Tool*)), this, SLOT(indicateToolIsCurrent(Tool*)));

  manager_->initialize();

  // Periodically process events for the splash screen.
  QCoreApplication::processEvents();

  if (display_config_file != "") {
    loadDisplayConfig(display_config_file);
  } else {
    loadDisplayConfig(QString::fromStdString(default_display_config_file_));
  }

  // Periodically process events for the splash screen.
  QCoreApplication::processEvents();

  delete splash_;
  splash_ = nullptr;

  initialized_ = true;
  Q_EMIT statusUpdate("RViz is ready.");

  connect(manager_, SIGNAL(preUpdate()), this, SLOT(updateFps()));
  connect(
    manager_, SIGNAL(statusUpdate(const QString&)), this,
    SIGNAL(statusUpdate(const QString&)));
}

VisualizationManager *
VisualizationFrame::getManager()
{
  return manager_;
}

void VisualizationFrame::initConfigs()
{
  home_dir_ = QDir::toNativeSeparators(QDir::homePath()).toStdString();

  config_dir_ = "";
  if (home_dir_ != "") {
    config_dir_ += home_dir_ + "/";
  }
  config_dir_ += ".rviz2";
  persistent_settings_file_ = config_dir_ + "/persistent_settings";
  default_display_config_file_ = config_dir_ + "/default." CONFIG_EXTENSION;

  QFile config_dir_as_file(QString::fromStdString(config_dir_));
  QDir config_dir_as_dir(QString::fromStdString(config_dir_));
  if (config_dir_as_file.exists() && !config_dir_as_dir.exists()) {
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "Moving file [" << config_dir_.c_str() << "] out of the way to recreate it as a directory.");
    std::string backup_file = config_dir_ + ".bak";

    if (!config_dir_as_file.rename(QString::fromStdString(backup_file))) {
      RVIZ_COMMON_LOG_ERROR("Failed to rename config directory while backing up.");
    }
  }

  QDir config_dir_as_qdir;
  if (!config_dir_as_qdir.mkpath(QString::fromStdString(config_dir_))) {
    RVIZ_COMMON_LOG_ERROR_STREAM("failed to make config dir: " << config_dir_);
  }
}

void VisualizationFrame::loadPersistentSettings()
{
  YamlConfigReader reader;
  Config config;
  reader.readFile(config, QString::fromStdString(persistent_settings_file_));
  if (!reader.error()) {
    QString last_config_dir, last_image_dir;
    if (config.mapGetString("Last Config Dir", &last_config_dir) &&
      config.mapGetString("Last Image Dir", &last_image_dir))
    {
      last_config_dir_ = last_config_dir.toStdString();
      last_image_dir_ = last_image_dir.toStdString();
    }

    Config recent_configs_list = config.mapGetChild("Recent Configs");
    recent_configs_.clear();
    int num_recent = recent_configs_list.listLength();
    for (int i = 0; i < num_recent; i++) {
      recent_configs_.push_back(
        recent_configs_list.listChildAt(
          i).getValue().toString().toStdString());
    }
  } else {
    RVIZ_COMMON_LOG_ERROR(qPrintable(reader.errorMessage()));
  }
}

void VisualizationFrame::savePersistentSettings()
{
  Config config;
  config.mapSetValue("Last Config Dir", QString::fromStdString(last_config_dir_));
  config.mapSetValue("Last Image Dir", QString::fromStdString(last_image_dir_));
  Config recent_configs_list = config.mapMakeChild("Recent Configs");
  for (D_string::iterator it = recent_configs_.begin(); it != recent_configs_.end(); ++it) {
    recent_configs_list.listAppendNew().setValue(QString::fromStdString(*it));
  }

  YamlConfigWriter writer;
  writer.writeFile(config, QString::fromStdString(persistent_settings_file_));

  if (writer.error()) {
    RVIZ_COMMON_LOG_ERROR(qPrintable(writer.errorMessage()));
  }
}

void VisualizationFrame::initMenus()
{
  file_menu_ = menuBar()->addMenu("&File");

  QAction * file_menu_open_action = file_menu_->addAction(
    "&Open Config", this, SLOT(
      onOpen()), QKeySequence("Ctrl+O"));
  this->addAction(file_menu_open_action);
  QAction * file_menu_save_action = file_menu_->addAction(
    "&Save Config", this, SLOT(
      onSave()), QKeySequence("Ctrl+S"));
  this->addAction(file_menu_save_action);
  QAction * file_menu_save_as_action =
    file_menu_->addAction(
    "Save Config &As", this, SLOT(onSaveAs()),
    QKeySequence("Ctrl+Shift+S"));
  this->addAction(file_menu_save_as_action);

  recent_configs_menu_ = file_menu_->addMenu("&Recent Configs");
  file_menu_->addAction("Save &Image", this, SLOT(onSaveImage()));
  if (show_choose_new_master_option_) {
    file_menu_->addSeparator();
    file_menu_->addAction("Change &Master", this, SLOT(changeMaster()));
  }
  file_menu_->addSeparator();

  QAction * file_menu_quit_action = file_menu_->addAction(
    "&Quit", this, SLOT(
      close()), QKeySequence("Ctrl+Q"));
  this->addAction(file_menu_quit_action);

  view_menu_ = menuBar()->addMenu("&Panels");
  view_menu_->addAction("Add &New Panel", this, SLOT(openNewPanelDialog()));
  delete_view_menu_ = view_menu_->addMenu("&Delete Panel");
  delete_view_menu_->setEnabled(false);

  QAction * fullscreen_action = view_menu_->addAction(
    "&Fullscreen", this, SLOT(
      setFullScreen(bool)), Qt::Key_F11);
  fullscreen_action->setCheckable(true);
  this->addAction(fullscreen_action);  // Also add to window, or the shortcut doest work
                                       // when the menu is hidden.
  connect(this, SIGNAL(fullScreenChange(bool)), fullscreen_action, SLOT(setChecked(bool)));
  new QShortcut(Qt::Key_Escape, this, SLOT(exitFullScreen()));
  view_menu_->addSeparator();

  QMenu * help_menu = menuBar()->addMenu("&Help");
  help_menu->addAction("Show &Help panel", this, SLOT(showHelpPanel()));
  help_menu->addSeparator();
  help_menu->addAction("&About", this, SLOT(onHelpAbout()));
}

void VisualizationFrame::initToolbars()
{
  QFont font;
  font.setPointSize(font.pointSizeF() * 0.9);

  // make toolbar with plugin tools

  toolbar_ = addToolBar("Tools");
  toolbar_->setFont(font);
  toolbar_->setContentsMargins(0, 0, 0, 0);
  toolbar_->setObjectName("Tools");
  toolbar_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  toolbar_actions_ = new QActionGroup(this);
  connect(
    toolbar_actions_, SIGNAL(triggered(QAction*)), this,
    SLOT(onToolbarActionTriggered(QAction*)));
  view_menu_->addAction(toolbar_->toggleViewAction());

  add_tool_action_ = new QAction("", toolbar_actions_);
  add_tool_action_->setToolTip("Add a new tool");
  add_tool_action_->setIcon(loadPixmap("package://rviz_common/icons/plus.png"));
  toolbar_->addAction(add_tool_action_);
  connect(add_tool_action_, SIGNAL(triggered()), this, SLOT(openNewToolDialog()));

  remove_tool_menu_ = new QMenu();
  QToolButton * remove_tool_button = new QToolButton();
  remove_tool_button->setMenu(remove_tool_menu_);
  remove_tool_button->setPopupMode(QToolButton::InstantPopup);
  remove_tool_button->setToolTip("Remove a tool from the toolbar");
  remove_tool_button->setIcon(loadPixmap("package://rviz_common/icons/minus.png"));
  toolbar_->addWidget(remove_tool_button);
  connect(
    remove_tool_menu_, SIGNAL(triggered(QAction*)), this, SLOT(
      onToolbarRemoveTool(QAction*)));
}

void VisualizationFrame::hideDockImpl(Qt::DockWidgetArea area, bool hide)
{
  QList<PanelDockWidget *> dock_widgets = findChildren<PanelDockWidget *>();

  for (QList<PanelDockWidget *>::iterator it = dock_widgets.begin(); it != dock_widgets.end();
    it++)
  {
    Qt::DockWidgetArea curr_area = dockWidgetArea(*it);
    if (area == curr_area) {
      (*it)->setCollapsed(hide);
    }
    // allow/disallow docking to this area for all widgets
    if (hide) {
      (*it)->setAllowedAreas((*it)->allowedAreas() & ~area);
    } else {
      (*it)->setAllowedAreas((*it)->allowedAreas() | area);
    }
  }
}

void VisualizationFrame::setHideButtonVisibility(bool visible)
{
  hide_left_dock_button_->setVisible(visible);
  hide_right_dock_button_->setVisible(visible);
}

void VisualizationFrame::hideLeftDock(bool hide)
{
  hideDockImpl(Qt::LeftDockWidgetArea, hide);
  hide_left_dock_button_->setArrowType(hide ? Qt::RightArrow : Qt::LeftArrow);
}

void VisualizationFrame::hideRightDock(bool hide)
{
  hideDockImpl(Qt::RightDockWidgetArea, hide);
  hide_right_dock_button_->setArrowType(hide ? Qt::LeftArrow : Qt::RightArrow);
}

void VisualizationFrame::onDockPanelVisibilityChange(bool visible)
{
  // if a dock widget becomes visible and is resting inside the
  // left or right dock area, we want to unhide the whole area
  if (visible) {
    QDockWidget * dock_widget = dynamic_cast<QDockWidget *>(sender());
    if (dock_widget) {
      Qt::DockWidgetArea area = dockWidgetArea(dock_widget);
      if (area == Qt::LeftDockWidgetArea) {
        hide_left_dock_button_->setChecked(false);
      }
      if (area == Qt::RightDockWidgetArea) {
        hide_right_dock_button_->setChecked(false);
      }
    }
  }
}

void VisualizationFrame::openNewPanelDialog()
{
  QString class_id;
  QString display_name;
  QStringList empty;

  NewObjectDialog * dialog = new NewObjectDialog(
    panel_factory_,
    "Panel",
    empty,
    empty,
    &class_id,
    &display_name,
    this);
  if (dialog->exec() == QDialog::Accepted) {
    addPanelByName(display_name, class_id);
  }
}

void VisualizationFrame::openNewToolDialog()
{
  QString class_id;
  QStringList empty;
  ToolManager * tool_man = manager_->getToolManager();

  NewObjectDialog * dialog = new NewObjectDialog(
    tool_man->getFactory(),
    "Tool",
    empty,
    tool_man->getToolClasses(),
    &class_id);
  if (dialog->exec() == QDialog::Accepted) {
    tool_man->addTool(class_id);
  }
  activateWindow();  // Force keyboard focus back on main window.
}

void VisualizationFrame::updateRecentConfigMenu()
{
  recent_configs_menu_->clear();

  D_string::iterator it = recent_configs_.begin();
  D_string::iterator end = recent_configs_.end();
  for (; it != end; ++it) {
    if (*it != "") {
      std::string display_name = *it;
      if (display_name == default_display_config_file_) {
        display_name += " (default)";
      }
      if (display_name.find(home_dir_) == 0) {
        display_name = (
          QDir::homePath() + "/" +
          QString::fromStdString(display_name.substr(home_dir_.size()))
        ).toStdString();
      }
      QString qdisplay_name = QString::fromStdString(display_name);
      QAction * action = new QAction(qdisplay_name, this);
      action->setData(QString::fromStdString(*it));
      connect(action, SIGNAL(triggered()), this, SLOT(onRecentConfigSelected()));
      recent_configs_menu_->addAction(action);
    }
  }
}

void VisualizationFrame::markRecentConfig(const std::string & path)
{
  D_string::iterator it = std::find(recent_configs_.begin(), recent_configs_.end(), path);
  if (it != recent_configs_.end()) {
    recent_configs_.erase(it);
  }

  recent_configs_.push_front(path);

  if (recent_configs_.size() > RECENT_CONFIG_COUNT) {
    recent_configs_.pop_back();
  }

  updateRecentConfigMenu();
}

void VisualizationFrame::loadDisplayConfig(const QString & qpath)
{
  std::string path = qpath.toStdString();
  QFileInfo path_info(qpath);
  std::string actual_load_path = path;
  if (!path_info.exists() || path_info.isDir()) {
    actual_load_path = package_path_ + "/default.rviz";
    if (!QFile(QString::fromStdString(actual_load_path)).exists()) {
      RVIZ_COMMON_LOG_ERROR_STREAM(
        "Default display config '" <<
          actual_load_path.c_str() << "' not found.  RViz will be very empty at first.");
      return;
    }
  }

  // Check if we have unsaved changes to the current config the same
  // as we do during exit, with the same option to cancel.
  if (!prepareToExit()) {
    return;
  }

  setWindowModified(false);
  loading_ = true;

  std::unique_ptr<LoadingDialog> dialog;
  if (initialized_) {
    dialog.reset(new LoadingDialog(this));
    dialog->show();
    connect(
      this, SIGNAL(statusUpdate(const QString&)),
      dialog.get(), SLOT(showMessage(const QString&)));
  }

  YamlConfigReader reader;
  Config config;
  reader.readFile(config, QString::fromStdString(actual_load_path));
  if (!reader.error()) {
    try {
      load(config);
    } catch (const std::exception & e) {
      RVIZ_COMMON_LOG_ERROR_STREAM("Could not load display config: " << e.what());
    }
  }

  markRecentConfig(path);

  setDisplayConfigFile(path);

  last_config_dir_ = path_info.absolutePath().toStdString();

  post_load_timer_->start(1000);
}

void VisualizationFrame::markLoadingDone()
{
  loading_ = false;
}

void VisualizationFrame::setImageSaveDirectory(const QString & directory)
{
  last_image_dir_ = directory.toStdString();
}

void VisualizationFrame::setDisplayConfigModified()
{
  if (!loading_) {
    if (!isWindowModified()) {
      setWindowModified(true);
    }
  }
}

void VisualizationFrame::setDisplayTitleFormat(const QString & title_format)
{
  display_title_format_ = title_format.toStdString();
}

void VisualizationFrame::setDisplayConfigFile(const std::string & path)
{
  display_config_file_ = path;
  std::string title;

  if (display_title_format_.empty()) {
    if (path == default_display_config_file_) {
      title = "RViz[*]";
    } else {
      title = QDir::toNativeSeparators(QString::fromStdString(path)).toStdString() + "[*] - RViz";
    }
  } else {
    auto find_and_replace_token =
      [](std::string & title, const std::string & token, const std::string & replacement)
      {
        std::size_t found = title.find(token);
        if (found != std::string::npos) {
          title.replace(found, token.length(), replacement);
        }
      };
    title = display_title_format_;
    std::filesystem::path full_filename(path.c_str());
    find_and_replace_token(
      title, "{NAMESPACE}",
      rviz_ros_node_.lock()->get_raw_node()->get_namespace());
    find_and_replace_token(title, "{CONFIG_PATH}", full_filename.parent_path().string());
    find_and_replace_token(title, "{CONFIG_FILENAME}", full_filename.filename().string());
    if (title.find("[*]") == std::string::npos) {
      title.append("[*]");
    }
  }

  setWindowTitle(QString::fromStdString(title));
}

bool VisualizationFrame::saveDisplayConfig(const QString & path)
{
  Config config;
  save(config);

  YamlConfigWriter writer;
  writer.writeFile(config, path);

  if (writer.error()) {
    RVIZ_COMMON_LOG_ERROR(qPrintable(writer.errorMessage()));
    error_message_ = writer.errorMessage();
    return false;
  } else {
    setWindowModified(false);
    error_message_ = "";
    return true;
  }
}

QString VisualizationFrame::getErrorMessage() const
{
  return error_message_;
}

void VisualizationFrame::save(Config config)
{
  manager_->save(config.mapMakeChild("Visualization Manager"));
  savePanels(config.mapMakeChild("Panels"));
  saveWindowGeometry(config.mapMakeChild("Window Geometry"));
}

void VisualizationFrame::load(const Config & config)
{
  manager_->load(config.mapGetChild("Visualization Manager"));
  loadPanels(config.mapGetChild("Panels"));
  loadWindowGeometry(config.mapGetChild("Window Geometry"));
}

void VisualizationFrame::loadWindowGeometry(const Config & config)
{
  int x, y;
  if (config.mapGetInt("X", &x) &&
    config.mapGetInt("Y", &y))
  {
    move(x, y);
  }

  int width, height;
  if (config.mapGetInt("Width", &width) &&
    config.mapGetInt("Height", &height))
  {
    resize(width, height);
  }

  QString main_window_config;
  if (config.mapGetString("QMainWindow State", &main_window_config)) {
    restoreState(QByteArray::fromHex(qPrintable(main_window_config)));
  }

  // load panel dock widget states (collapsed or not)
  QList<PanelDockWidget *> dock_widgets = findChildren<PanelDockWidget *>();

  for (QList<PanelDockWidget *>::iterator it = dock_widgets.begin(); it != dock_widgets.end();
    it++)
  {
    Config itConfig = config.mapGetChild((*it)->windowTitle());

    if (itConfig.isValid()) {
      (*it)->load(itConfig);
    }
  }

  bool b;
  config.mapGetBool("Hide Left Dock", &b);
  hide_left_dock_button_->setChecked(b);
  hideLeftDock(b);
  config.mapGetBool("Hide Right Dock", &b);
  hideRightDock(b);
  hide_right_dock_button_->setChecked(b);
}

void VisualizationFrame::saveWindowGeometry(Config config)
{
  config.mapSetValue("X", x());
  config.mapSetValue("Y", y());
  config.mapSetValue("Width", width());
  config.mapSetValue("Height", height());

  QByteArray window_state = saveState().toHex();
  config.mapSetValue("QMainWindow State", window_state.constData());

  config.mapSetValue("Hide Left Dock", hide_left_dock_button_->isChecked());
  config.mapSetValue("Hide Right Dock", hide_right_dock_button_->isChecked());

  // save panel dock widget states (collapsed or not)
  QList<PanelDockWidget *> dock_widgets = findChildren<PanelDockWidget *>();

  for (QList<PanelDockWidget *>::iterator it = dock_widgets.begin(); it != dock_widgets.end();
    it++)
  {
    (*it)->save(config.mapMakeChild((*it)->windowTitle()));
  }
}

void VisualizationFrame::loadPanels(const Config & config)
{
  // First destroy any existing custom panels.
  for (int i = 0; i < custom_panels_.size(); i++) {
    delete custom_panels_[i].dock;
    delete custom_panels_[i].delete_action;
  }
  custom_panels_.clear();

  // Then load the ones in the config.
  int num_custom_panels = config.listLength();
  for (int i = 0; i < num_custom_panels; i++) {
    Config panel_config = config.listChildAt(i);

    QString class_id, name;
    if (panel_config.mapGetString("Class", &class_id) &&
      panel_config.mapGetString("Name", &name))
    {
      QDockWidget * dock = addPanelByName(name, class_id);
      // This is kind of ridiculous - should just be something like
      // createPanel() and addPanel() so I can do load() without this
      // qobject_cast.
      if (dock) {
        Panel * panel = qobject_cast<Panel *>(dock->widget());
        if (panel) {
          panel->load(panel_config);
        }
      }
    }
  }
}

void VisualizationFrame::savePanels(Config config)
{
  // Not really necessary, but gives an empty list if there are no entries,
  // instead of an Empty config node.
  config.setType(Config::List);

  for (int i = 0; i < custom_panels_.size(); i++) {
    custom_panels_[i].panel->save(config.listAppendNew());
  }
}

bool VisualizationFrame::prepareToExit()
{
  if (!initialized_) {
    return true;
  }

  savePersistentSettings();

  if (isWindowModified()) {
    QMessageBox box(this);
    box.setText("There are unsaved changes.");
    box.setInformativeText(QString::fromStdString("Save changes to " + display_config_file_ + "?"));
    box.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Save);
    int result = box.exec();
    switch (result) {
      case QMessageBox::Save:
        if (saveDisplayConfig(QString::fromStdString(display_config_file_))) {
          return true;
        } else {
          QMessageBox box(this);
          box.setWindowTitle("Failed to save.");
          box.setText(getErrorMessage());
          box.setInformativeText(
            QString::fromStdString(
              "Save copy of " + display_config_file_ + " to another file?"));
          box.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
          box.setDefaultButton(QMessageBox::Save);
          int result = box.exec();
          switch (result) {
            case QMessageBox::Save:
              onSaveAs();
              return true;
            case QMessageBox::Discard:
              return true;
            default:
              return false;
          }
        }
      case QMessageBox::Discard:
        return true;
      default:
        return false;
    }
  } else {
    return true;
  }
}

void VisualizationFrame::onOpen()
{
  QString filename = QFileDialog::getOpenFileName(
    this, "Choose a file to open",
    QString::fromStdString(last_config_dir_),
    "RViz config files (" CONFIG_EXTENSION_WILDCARD ")");

  if (!filename.isEmpty()) {
    if (!QFile(filename).exists()) {
      QString message = filename + " does not exist!";
      QMessageBox::critical(this, "Config file does not exist", message);
      return;
    }

    loadDisplayConfig(filename);
  }
}

void VisualizationFrame::onSave()
{
  if (!initialized_) {
    return;
  }

  savePersistentSettings();

  if (!saveDisplayConfig(QString::fromStdString(display_config_file_))) {
    QMessageBox box(this);
    box.setWindowTitle("Failed to save.");
    box.setText(getErrorMessage());
    box.setInformativeText(
      QString::fromStdString(
        "Save copy of " + display_config_file_ + " to another file?"));
    box.setStandardButtons(QMessageBox::Save | QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Save);
    if (box.exec() == QMessageBox::Save) {
      onSaveAs();
    }
  }
}

void VisualizationFrame::onSaveAs()
{
  QString q_filename = QFileDialog::getSaveFileName(
    this, "Choose a file to save to",
    QString::fromStdString(last_config_dir_),
    "RViz config files (" CONFIG_EXTENSION_WILDCARD ")");

  if (!q_filename.isEmpty()) {
    if (!q_filename.endsWith("." CONFIG_EXTENSION)) {
      q_filename += "." CONFIG_EXTENSION;
    }

    if (!saveDisplayConfig(q_filename)) {
      QMessageBox::critical(this, "Failed to save.", getErrorMessage());
    }

    std::string filename = q_filename.toStdString();
    markRecentConfig(filename);
    last_config_dir_ = QDir(q_filename).dirName().toStdString();
    setDisplayConfigFile(filename);
  }
}

void VisualizationFrame::onSaveImage()
{
  ScreenshotDialog * dialog =
    new ScreenshotDialog(this, render_panel_, QString::fromStdString(last_image_dir_));
  connect(
    dialog, SIGNAL(savedInDirectory(const QString&)),
    this, SLOT(setImageSaveDirectory(const QString&)));
  dialog->show();
}

void VisualizationFrame::onRecentConfigSelected()
{
  QAction * action = dynamic_cast<QAction *>(sender());
  if (action) {
    QString path = action->data().toString();
    if (path.size() != 0) {
      if (!QFile(path).exists()) {
        QString message = path + " does not exist!";
        QMessageBox::critical(this, "Config file does not exist", message);
        return;
      }

      loadDisplayConfig(path);
    }
  }
}

void VisualizationFrame::addTool(Tool * tool)
{
  QAction * action = new QAction(tool->getName(), toolbar_actions_);
  action->setIcon(tool->getIcon());
  action->setIconText(tool->getName());
  action->setCheckable(true);
  toolbar_->insertAction(add_tool_action_, action);
  action_to_tool_map_[action] = tool;
  tool_to_action_map_[tool] = action;

  remove_tool_menu_->addAction(tool->getName());

  QObject::connect(
    tool, &Tool::nameChanged, this,
    &VisualizationFrame::VisualizationFrame::onToolNameChanged);
}

void VisualizationFrame::onToolNameChanged(const QString & name)
{
  // Early return if the tool is not present
  auto it = tool_to_action_map_.find(qobject_cast<Tool *>(sender()));
  if (it == tool_to_action_map_.end()) {
    return;
  }

  // Change the name of the action
  it->second->setIconText(name);
}

void VisualizationFrame::onToolbarActionTriggered(QAction * action)
{
  Tool * tool = action_to_tool_map_[action];

  if (tool) {
    manager_->getToolManager()->setCurrentTool(tool);
  }
}

void VisualizationFrame::onToolbarRemoveTool(QAction * remove_tool_menu_action)
{
  QString name = remove_tool_menu_action->text();
  for (int i = 0; i < manager_->getToolManager()->numTools(); i++) {
    Tool * tool = manager_->getToolManager()->getTool(i);
    if (tool->getName() == name) {
      manager_->getToolManager()->removeTool(i);
      return;
    }
  }
}

void VisualizationFrame::removeTool(Tool * tool)
{
  QAction * action = tool_to_action_map_[tool];
  if (action) {
    toolbar_actions_->removeAction(action);
    toolbar_->removeAction(action);
    tool_to_action_map_.erase(tool);
    action_to_tool_map_.erase(action);
  }
  QString tool_name = tool->getName();
  QList<QAction *> remove_tool_actions = remove_tool_menu_->actions();
  for (int i = 0; i < remove_tool_actions.size(); i++) {
    QAction * removal_action = remove_tool_actions.at(i);
    if (removal_action->text() == tool_name) {
      remove_tool_menu_->removeAction(removal_action);
      break;
    }
  }
}

void VisualizationFrame::refreshTool(Tool * tool)
{
  QAction * action = tool_to_action_map_[tool];
  action->setIcon(tool->getIcon());
  action->setIconText(tool->getName());
}

void VisualizationFrame::indicateToolIsCurrent(Tool * tool)
{
  QAction * action = tool_to_action_map_[tool];
  if (action) {
    action->setChecked(true);
  }
}

void VisualizationFrame::showHelpPanel()
{
  if (!show_help_action_) {
    QDockWidget * dock = addPanelByName("Help", "rviz_common/Help");
    show_help_action_ = dock->toggleViewAction();
    connect(dock, SIGNAL(destroyed(QObject*)), this, SLOT(onHelpDestroyed()));
  } else {
    show_help_action_->trigger();
  }
}

void VisualizationFrame::onHelpDestroyed()
{
  show_help_action_ = nullptr;
}

void VisualizationFrame::onHelpAbout()
{
  QString about_text = QString(
    "This is RViz version %1 (%2).\n"
    "\n"
    "Compiled against Qt version %3."
    "\n"
    "Compiled against OGRE version %4.%5.%6%7 (%8).")
    .arg(get_version().c_str())
    .arg(get_distro().c_str())
    .arg(QT_VERSION_STR)
    .arg(OGRE_VERSION_MAJOR)
    .arg(OGRE_VERSION_MINOR)
    .arg(OGRE_VERSION_PATCH)
    .arg(OGRE_VERSION_SUFFIX)
    .arg(OGRE_VERSION_NAME);

  QMessageBox::about(QApplication::activeWindow(), "About", about_text);
}

QWidget * VisualizationFrame::getParentWindow()
{
  return this;
}

void VisualizationFrame::onDeletePanel()
{
  // This should only be called as a SLOT from a QAction in the
  // "delete panel" submenu, so the sender will be one of the QActions
  // stored as "delete_action" in a PanelRecord.  This code looks for
  // a delete_action in custom_panels_ matching sender() and removes
  // the panel associated with it.
  if (QAction * action = qobject_cast<QAction *>(sender())) {
    for (int i = 0; i < custom_panels_.size(); i++) {
      if (custom_panels_[i].delete_action == action) {
        delete custom_panels_[i].dock;
        custom_panels_.removeAt(i);
        setDisplayConfigModified();
        action->deleteLater();
        if (delete_view_menu_->actions().size() == 1 &&
          delete_view_menu_->actions().first() == action)
        {
          delete_view_menu_->setEnabled(false);
        }
        return;
      }
    }
  }
}

void VisualizationFrame::setFullScreen(bool full_screen)
{
  auto state = windowState();
  if (full_screen == state.testFlag(Qt::WindowFullScreen)) {
    return;
  }
  Q_EMIT (fullScreenChange(full_screen));

  // When switching to fullscreen, remember visibility state of toolbar
  if (full_screen) {
    toolbar_visible_ = toolbar_->isVisible();
  }
  menuBar()->setVisible(!full_screen);
  toolbar_->setVisible(!full_screen && toolbar_visible_);
  statusBar()->setVisible(!full_screen);
  setHideButtonVisibility(!full_screen);

  if (full_screen) {
    setWindowState(state | Qt::WindowFullScreen);
  } else {
    setWindowState(state & ~Qt::WindowFullScreen);
  }
  show();
}

void VisualizationFrame::exitFullScreen()
{
  setFullScreen(false);
}

QDockWidget * VisualizationFrame::addPanelByName(
  const QString & name,
  const QString & class_id,
  Qt::DockWidgetArea area,
  bool floating)
{
  QString error;
  Panel * panel = panel_factory_->make(class_id, &error);
  if (!panel) {
    panel = new FailedPanel(class_id, error);
  }
  panel->setName(name);
  connect(panel, SIGNAL(configChanged()), this, SLOT(setDisplayConfigModified()));

  PanelRecord record;
  record.dock = addPane(name, panel, area, floating);
  record.panel = panel;
  record.name = name;
  record.delete_action = delete_view_menu_->addAction(name, this, SLOT(onDeletePanel()));
  custom_panels_.append(record);
  delete_view_menu_->setEnabled(true);

  record.panel->initialize(manager_);

  record.dock->setIcon(panel_factory_->getPluginInfo(class_id).icon);

  return record.dock;
}

PanelDockWidget * VisualizationFrame::addPane(
  const QString & name, QWidget * panel,
  Qt::DockWidgetArea area, bool floating)
{
  PanelDockWidget * dock;
  dock = new PanelDockWidget(name);
  dock->setContentWidget(panel);
  dock->setFloating(floating);
  dock->setObjectName(name);   // QMainWindow::saveState() needs objectName to be set.
  addDockWidget(area, dock);

  // we want to know when that panel becomes visible
  connect(dock, SIGNAL(visibilityChanged(bool)), this, SLOT(onDockPanelVisibilityChange(bool)));
  connect(this, SIGNAL(fullScreenChange(bool)), dock, SLOT(overrideVisibility(bool)));

  QAction * toggle_action = dock->toggleViewAction();
  view_menu_->addAction(toggle_action);

  connect(toggle_action, SIGNAL(triggered(bool)), this, SLOT(setDisplayConfigModified()));
  connect(dock, SIGNAL(closed()), this, SLOT(setDisplayConfigModified()));

  dock->installEventFilter(geom_change_detector_);

  // repair/update visibility status
  hideLeftDock(area == Qt::LeftDockWidgetArea ? false : hide_left_dock_button_->isChecked());
  hideRightDock(area == Qt::RightDockWidgetArea ? false : hide_right_dock_button_->isChecked());

  return dock;
}

}  // namespace rviz_common
