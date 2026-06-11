// Copyright (c) 2024, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Consolidated pluginlib exports for all view controllers.
// Keeping PLUGINLIB_EXPORT_CLASS in a single TU means pluginlib/class_list_macros.hpp
// (~347 headers) is parsed exactly once instead of once per view controller .cpp file.

#include "rviz_default_plugins/view_controllers/follower/third_person_follower_view_controller.hpp"
#include "rviz_default_plugins/view_controllers/fps/fps_view_controller.hpp"
#include "rviz_default_plugins/view_controllers/frame/frame_view_controller.hpp"
#include "rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp"
#include "rviz_default_plugins/view_controllers/ortho/fixed_orientation_ortho_view_controller.hpp"
#include "rviz_default_plugins/view_controllers/xy_orbit/xy_orbit_view_controller.hpp"

#include <pluginlib/class_list_macros.hpp>  // NOLINT(build/include_order)

PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::ThirdPersonFollowerViewController,
  rviz_common::ViewController)
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::FPSViewController, rviz_common::ViewController)
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::FrameViewController, rviz_common::ViewController)
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::OrbitViewController, rviz_common::ViewController)
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::FixedOrientationOrthoViewController,
  rviz_common::ViewController)
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::XYOrbitViewController,
  rviz_common::ViewController)
