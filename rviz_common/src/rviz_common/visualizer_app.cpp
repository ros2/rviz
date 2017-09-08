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

#include <OgreGpuProgramManager.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreMaterialManager.h>

#include <QApplication>
#include <QTimer>

#include <iostream>

#ifdef Q_OS_MAC
#include <ApplicationServices/ApplicationServices.h>
// Apparently OSX #defines 'check' to be an empty string somewhere.
// That was fun to figure out.
#undef check
#endif

#include "rviz_common/logging.hpp"
#include "rviz_common/ros_integration/init.hpp"
#include "rviz_common/ros_integration/ok.hpp"

#include "./visualization_frame.hpp"
// #include "./visualization_manager.hpp"

// #include "rviz/env_config.h"
// #include "rviz/ogre_helpers/ogre_logging.h"
// #include "rviz/ogre_helpers/render_system.h"
// #include "rviz/selection/selection_manager.h"
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

VisualizerApp::VisualizerApp()
: app_(0), continue_timer_(0), frame_(0), node_name_("")
{}

void VisualizerApp::setApp(QApplication * app)
{
  app_ = app;
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

#ifdef Q_OS_MAC
  ProcessSerialNumber PSN;
  GetCurrentProcess(&PSN);
  TransformProcessType(&PSN, kProcessTransformToForegroundApplication);
  SetFrontProcess(&PSN);
#endif
  
  // TODO(jsquare): remove line below after there is a implementation for rviz_common::ros_integration::init and the include of iostream
  std::cout << argc << argv;
  // node_name_ = rviz_common::ros_integration::init(argc, argv, "rviz", true /* anonymous_name */);

  startContinueChecker();

  // rviz_common::install_rviz_rendering_log_handlers();

  // TODO(wjwwood): restore the program options without using Boost
  // po::options_description options;
  // options.add_options()
  //   ("help,h", "Produce this help message")
  //   ("splash-screen,s", po::value<std::string>(), "A custom splash-screen image to display")
  //   ("help-file", po::value<std::string>(), "A custom html file to show as the help screen")
  //   ("display-config,d", po::value<std::string>(), "A display config file (.rviz) to load")
  //   ("fixed-frame,f", po::value<std::string>(), "Set the fixed frame")
  //   ("ogre-log,l", "Enable the Ogre.log file (output in cwd) and console output.")
  //   ("in-mc-wrapper", "Signal that this is running inside a master-chooser wrapper")
  //   ("opengl", po::value<int>(),
  //   "Force OpenGL version (use '--opengl 210' for OpenGL 2.1 compatibility mode)")
  //   ("disable-anti-aliasing", "Prevent rviz from trying to use anti-aliasing when rendering.")
  //   ("no-stereo", "Disable the use of stereo rendering.")
  //   ("verbose,v", "Enable debug visualizations")
  //   ("log-level-debug", "Sets the ROS logger level to debug.");
  // po::variables_map vm;
  // std::string display_config, fixed_frame, splash_path, help_path;
  // bool enable_ogre_log = false;
  // bool in_mc_wrapper = false;
  // bool verbose = false;
  // int force_gl_version = 0;
  // bool disable_anti_aliasing = false;
  // bool disable_stereo = false;
  // try {
  //   po::store(po::parse_command_line(argc, argv, options), vm);
  //   po::notify(vm);
  //
  //   if (vm.count("help")) {
  //     std::cout << "rviz command line options:\n" << options;
  //     return false;
  //   }
  //
  //   if (vm.count("in-mc-wrapper")) {
  //     in_mc_wrapper = true;
  //   }
  //
  //   if (vm.count("display-config")) {
  //     display_config = vm["display-config"].as<std::string>();
  //     if (display_config.substr(display_config.size() - 4, 4) == ".vcg") {
  //       std::cerr << "ERROR: the config file '" << display_config <<
  //         "' is a .vcg file, which is the old rviz config format." << std::endl;
  //       std::cerr <<
  //         "       New config files have a .rviz extension and use YAML formatting.  "
  //         "The format changed"
  //                 <<
  //         std::endl;
  //       std::cerr <<
  //         "       between Fuerte and Groovy.  "
  //         "There is not (yet) an automated conversion program."
  //                 <<
  //         std::endl;
  //       return false;
  //     }
  //   }
  //
  //   if (vm.count("splash-screen")) {
  //     splash_path = vm["splash-screen"].as<std::string>();
  //   }
  //
  //   if (vm.count("help-file")) {
  //     help_path = vm["help-file"].as<std::string>();
  //   }
  //
  //   if (vm.count("fixed-frame")) {
  //     fixed_frame = vm["fixed-frame"].as<std::string>();
  //   }
  //
  //   if (vm.count("ogre-log")) {
  //     enable_ogre_log = true;
  //   }
  //
  //   if (vm.count("no-stereo")) {
  //     disable_stereo = true;
  //   }
  //
  //   if (vm.count("opengl")) {
  //     //std::cout << vm["opengl"].as<std::string>() << std::endl;
  //     force_gl_version = vm["opengl"].as<int>();
  //   }
  //
  //   if (vm.count("disable-anti-aliasing")) {
  //     disable_anti_aliasing = true;
  //   }
  //
  //   if (vm.count("verbose")) {
  //     verbose = true;
  //   }
  //
  //   if (vm.count("log-level-debug")) {
  //     if (
  //       ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  //     {
  //       ros::console::notifyLoggerLevelsChanged();
  //     }
  //   }
  // } catch (std::exception & e) {
  //   ROS_ERROR("Error parsing command line: %s", e.what());
  //   return false;
  // }
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
  // if (enable_ogre_log) {
  //   OgreLogging::useRosLog();
  // }
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

  // frame_ = new VisualizationFrame();
  // frame_->setApp(this->app_);

  // if (help_path != "") {
  //   frame_->setHelpPath(QString::fromStdString(help_path));
  // }

  // TODO(wjwwood): figure out how to preserve the "choost new master" feature
  // frame_->setShowChooseNewMaster(in_mc_wrapper);

  // if (vm.count("splash-screen") ) {
  //   frame_->setSplashPath(QString::fromStdString(splash_path));
  // }

  // frame_->initialize(QString::fromStdString(display_config));
  // frame_->initialize();

  // if (!fixed_frame.empty() ) {
  //   frame_->getManager()->setFixedFrame(QString::fromStdString(fixed_frame));
  // }

  // frame_->getManager()->getSelectionManager()->setDebugMode(verbose);
  // frame_->getManager()->getSelectionManager()->setDebugMode(false);

  // frame_->show();

  // TODO(wjwwood): reenable the ROS service to reload the shaders via the ros_integration API
  // ros::NodeHandle private_nh("~");
  // reload_shaders_service_ = private_nh.advertiseService("reload_shaders", reloadShaders);

  return true;
}

VisualizerApp::~VisualizerApp()
{
  delete continue_timer_;
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
  if (node_name_ == "") {
    // If true, this implies the setup of the node has not occurred yet.
    // This should not happen.
    return;
  }
  // if (!rviz_common::ros_integration::ok(node_name_)) {
    if (frame_) {
      // Make sure the window doesn't ask if we want to save first.
      frame_->setWindowModified(false);
    }
    QApplication::closeAllWindows();
  // }
}

}  // namespace rviz_common
