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

namespace rviz_common
{

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
  rviz_common::install_rviz_rendering_log_handlers();

  QCommandLineParser parser;
  parser.setApplicationDescription("3D visualization tool for ROS2");
  parser.addHelpOption();

  QCommandLineOption display_title_format_option(
    QStringList() << "t" << "display-title-format",
      "A display title format like ",
      "\"{NAMESPACE} - {CONFIG_PATH}/{CONFIG_FILENAME} - RViz2\" ",
      "display_title_format");
  parser.addOption(display_title_format_option);

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

  QCommandLineOption fullscreen_option(
    "fullscreen",
    "Start RViz in fullscreen mode.");
  parser.addOption(fullscreen_option);

  QString display_config, fixed_frame, splash_path, help_path, display_title_format;
  bool enable_ogre_log, fullscreen;

  if (app_) {parser.process(*app_);}

  enable_ogre_log = parser.isSet(ogre_log_option);
  fullscreen = parser.isSet(fullscreen_option);

  if (parser.isSet(display_config_option)) {
    display_config = parser.value(display_config_option);
  }
  if (parser.isSet(fixed_frame_option)) {
    fixed_frame = parser.value(fixed_frame_option);
  }

  if (parser.isSet(splash_screen_option)) {
    splash_path = parser.value(splash_screen_option);
  }

  if (parser.isSet(display_title_format_option)) {
    display_title_format = parser.value(display_title_format_option);
  }

  if (enable_ogre_log) {
    rviz_rendering::OgreLogging::get()->useLogFileAndStandardOut();
    rviz_rendering::OgreLogging::get()->configureLogging();
  }

  startContinueChecker();

  node_ = ros_client_abstraction_->init(argc, argv, "rviz", false /* anonymous_name */);

  frame_ = new VisualizationFrame(node_);

  frame_->setDisplayTitleFormat(display_title_format);

  frame_->setApp(this->app_);

  if (!help_path.isEmpty()) {
    frame_->setHelpPath(help_path);
  }

  if (!splash_path.isEmpty()) {
    frame_->setSplashPath(splash_path);
  }
  frame_->initialize(node_, display_config);

  if (!fixed_frame.isEmpty()) {
    frame_->getManager()->setFixedFrame(fixed_frame);
  }

  if (fullscreen) {
    frame_->setFullScreen(true);
  }

  frame_->show();

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
