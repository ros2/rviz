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

#include "rviz_common/visualizer_app.hpp"

#include <iostream>
#include <memory>
#include <utility>

// #include <OgreGpuProgramManager.h>
// #include <OgreHighLevelGpuProgramManager.h>
// #include <OgreMaterialManager.h>

#include <QApplication>  // NOLINT: cpplint is unable to handle the include order here
#include <QCommandLineParser>  // NOLINT: cpplint is unable to handle the include order here
#include <QCommandLineOption>  // NOLINT: cpplint is unable to handle the include order here
#include <QTimer>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_rendering/ogre_logging.hpp"

#include "rviz_common/visualization_frame.hpp"
#include "rviz_common/visualization_manager.hpp"

// TODO(wjwwood): figure out a non-depricated way to do this
#if 0
#ifdef Q_OS_MAC
#include <ApplicationServices/ApplicationServices.h>
// Apparently OSX #defines 'check' to be an empty string somewhere.
// That was fun to figure out.
#undef check
#endif
#endif

// #include "rviz/env_config.h"
// #include "rviz/ogre_helpers/ogre_logging.h"
// #include "rviz/ogre_helpers/render_system.h"
// #include "rviz/wait_for_master_dialog.h"

namespace rviz_common
{

// TODO(wjwwood): reenable the service to reload the shaders
// bool
// reloadShaders(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
// {
//   ROS_INFO("Reloading materials.");
//   {
//     Ogre::ResourceManager::ResourceMapIterator it =
//       Ogre::MaterialManager::getSingleton().getResourceIterator();
//     while (it.hasMoreElements()) {
//       Ogre::ResourcePtr resource = it.getNext();
//       resource->reload();
//     }
//   }
//   ROS_INFO("Reloading high-level gpu shaders.");
//   {
//     Ogre::ResourceManager::ResourceMapIterator it =
//       Ogre::HighLevelGpuProgramManager::getSingleton().getResourceIterator();
//     while (it.hasMoreElements()) {
//       Ogre::ResourcePtr resource = it.getNext();
//       resource->reload();
//     }
//   }
//   ROS_INFO("Reloading gpu shaders.");
//   {
//     Ogre::ResourceManager::ResourceMapIterator it =
//       Ogre::GpuProgramManager::getSingleton().getResourceIterator();
//     while (it.hasMoreElements()) {
//       Ogre::ResourcePtr resource = it.getNext();
//       resource->reload();
//     }
//   }
//   return true;
// }

VisualizerApp::VisualizerApp(
  std::unique_ptr<rviz_common::ros_integration::RosClientAbstractionIface> ros_client_abstraction)
: app_(0),
  continue_timer_(0),
  frame_(0),
  node_(),
  ros_client_abstraction_(std::move(ros_client_abstraction))
{}

void VisualizerApp::setApp(QApplication * app)
{
  app_ = app;
}

rviz_rendering::RenderWindow * VisualizerApp::getRenderWindow()
{
  return frame_->getRenderWindow();
}

void VisualizerApp::loadConfig(QString config_path)
{
  frame_->loadDisplayConfig(config_path);
}

bool VisualizerApp::init(int argc, char ** argv)
{
  // TODO(wjwwood): find a way to get the versions and print them here
  //                also include versions of more things, like rviz_rendering,
  //                rviz_common, and the plugins
  // RVIZ_COMMON_LOG_INFO_STREAM("rviz version " << get_version().c_str());
  // RVIZ_COMMON_LOG_INFO("compiled against Qt version " QT_VERSION_STR);
  // RVIZ_COMMON_LOG_INFO_STREAM(
  //   "compiled against OGRE version " <<
  //     OGRE_VERSION_MAJOR << "." <<
  //     OGRE_VERSION_MINOR << "." <<
  //     OGRE_VERSION_PATCH << OGRE_VERSION_SUFFIX <<
  //     " (" << OGRE_VERSION_NAME << ")");

// TODO(wjwwood): figure out a non-depricated way to do this
#if 0
#ifdef Q_OS_MAC
  ProcessSerialNumber PSN;
  GetCurrentProcess(&PSN);
  TransformProcessType(&PSN, kProcessTransformToForegroundApplication);
  SetFrontProcess(&PSN);
#endif
#endif

  rviz_common::install_rviz_rendering_log_handlers();

  QCommandLineParser parser;
  parser.setApplicationDescription("3D visualization tool for ROS2");
  parser.addHelpOption();

  QCommandLineOption display_config_option(
    QStringList() << "d" << "display-config",
      "A display config file (.rviz) to load",
      "display_config");
  parser.addOption(display_config_option);

  QCommandLineOption fixed_frame_option(
    QStringList() << "f" << "fixed-frame", "Set the fixed frame", "fixed_frame");
  parser.addOption(fixed_frame_option);

  QCommandLineOption ogre_log_option(
    QStringList() << "l" << "ogre-log",
      "Enable the Ogre.log file (output in cwd) and console output.");
  parser.addOption(ogre_log_option);

  QCommandLineOption splash_screen_option(
    QStringList() << "s" << "splash-screen",
      "A custom splash-screen image to display", "splash_path");
  parser.addOption(splash_screen_option);

// TODO(botteroa-si): enable when possible
//  QCommandLineOption help_file_option(
//    "help-file", "A custom html file to show as the help screen", "help_path");
//  parser.addOption(help_file_option);
//
//  QCommandLineOption open_gl_option(
//    "opengl",
//    "Force OpenGL version (use '--opengl 210' for OpenGL 2.1 compatibility mode)",
//    "version");
//  parser.addOption(open_gl_option);
//
//  QCommandLineOption disable_anti_aliasing_option(
//    "disable-anti-aliasing", "Prevent rviz from trying to use anti-aliasing when rendering.");
//  parser.addOption(disable_anti_aliasing_option);
//
//  QCommandLineOption no_stereo_option("no-stereo", "Disable the use of stereo rendering.");
//  parser.addOption(no_stereo_option);
//
//  QCommandLineOption log_level_debug_option(
//    "log-level-debug", "Sets the ROS logger level to debug.");
//  parser.addOption(log_level_debug_option);

//   ("in-mc-wrapper", "Signal that this is running inside a master-chooser wrapper")

  QString display_config, fixed_frame, splash_path, help_path;
  bool enable_ogre_log;
  // TODO(botteroa-si): enable when possible
//  bool in_mc_wrapper = false;
//  int force_gl_version = 0;
//  bool disable_anti_aliasing = false;
//  bool disable_stereo = false;

  parser.process(*app_);

  enable_ogre_log = parser.isSet(ogre_log_option);
//    disable_stereo = parser.isSet(no_stereo_option);
//    disable_anti_aliasing = parser.isSet(disable_anti_aliasing_option);

  if (parser.isSet(display_config_option)) {
    display_config = parser.value(display_config_option);
  }
  if (parser.isSet(fixed_frame_option)) {
    fixed_frame = parser.value(fixed_frame_option);
  }

  if (parser.isSet(splash_screen_option)) {
    splash_path = parser.value(splash_screen_option);
  }
// TODO(botteroa-si): enable when possible
//    if (parser.isSet(help_file_option)) {
//      help_path = parser.value(help_file_option);
//    }
//    if (parser.isSet(open_gl_option)) {
//      force_gl_version = parser.value(open_gl_option).toInt();
//    }

//   if (vm.count("in-mc-wrapper")) {
//     in_mc_wrapper = true;
//   }
//
//
//   if (vm.count("log-level-debug")) {
//     if (
//       ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
//     {
//       ros::console::notifyLoggerLevelsChanged();
//     }
//   }

  //
  // if (!ros::master::check() ) {
  // TODO(wjwwood): figure out how to support the "wait for master" functionality
  //                while also using the rviz_common/ros_integration abstraction
  //   WaitForMasterDialog * dialog = new WaitForMasterDialog;
  //   if (dialog->exec() != QDialog::Accepted) {
  //     return false;
  //   }
  // }
  //
  // nh_.reset(new ros::NodeHandle);
  //
  if (enable_ogre_log) {
    rviz_rendering::OgreLogging::get()->useLogFileAndStandardOut();
    rviz_rendering::OgreLogging::get()->configureLogging();
  }
  //
  // if (force_gl_version) {
  //   RenderSystem::forceGlVersion(force_gl_version);
  // }
  //
  // if (disable_anti_aliasing) {
  //   RenderSystem::disableAntiAliasing();
  // }
  //
  // if (disable_stereo) {
  //   RenderSystem::forceNoStereo();
  // }

  startContinueChecker();

  // TODO(wjwwood): anonymous is not working right now, reenable later
  // node_name_ = rviz_common::ros_integration::init(argc, argv, "rviz", true /* anonymous_name */);
  node_ = ros_client_abstraction_->init(argc, argv, "rviz", false /* anonymous_name */);

  frame_ = new VisualizationFrame(node_);
  frame_->setApp(this->app_);

  if (!help_path.isEmpty()) {
    frame_->setHelpPath(help_path);
  }

  // TODO(wjwwood): figure out how to preserve the "choost new master" feature
  // frame_->setShowChooseNewMaster(in_mc_wrapper);

  if (!splash_path.isEmpty()) {
    frame_->setSplashPath(splash_path);
  }
  frame_->initialize(node_, display_config);

  if (!fixed_frame.isEmpty()) {
    frame_->getManager()->setFixedFrame(fixed_frame);
  }

  frame_->show();

  // TODO(wjwwood): reenable the ROS service to reload the shaders via the ros_integration API
  // ros::NodeHandle private_nh("~");
  // reload_shaders_service_ = private_nh.advertiseService("reload_shaders", reloadShaders);

  return true;
}

VisualizerApp::~VisualizerApp()
{
  delete continue_timer_;
  ros_client_abstraction_->shutdown();
  delete frame_;
}

void VisualizerApp::startContinueChecker()
{
  continue_timer_ = new QTimer(this);
  connect(continue_timer_, SIGNAL(timeout()), this, SLOT(checkContinue()));
  continue_timer_->start(100);
}

void VisualizerApp::checkContinue()
{
  if (!ros_client_abstraction_->ok()) {
    if (frame_) {
      // Make sure the window doesn't ask if we want to save first.
      frame_->setWindowModified(false);
    }
    QApplication::closeAllWindows();
  }
}

}  // namespace rviz_common
