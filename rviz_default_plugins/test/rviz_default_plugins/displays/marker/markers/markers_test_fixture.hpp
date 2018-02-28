/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
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
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__MARKERS_TEST_FIXTURE_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__MARKERS_TEST_FIXTURE_HPP_

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <string>

#include <OgreMath.h>
#include <OgreRoot.h>

#include <QApplication>  // NOLINT

#include "test/rviz_rendering/ogre_testing_environment.hpp"
#include "../../display_test_fixture.hpp"
#include "../../../mock_display_context.hpp"
#include "../../../mock_frame_manager.hpp"
#include "../../../mock_selection_manager.hpp"
#include "../../../../../src/rviz_default_plugins/displays/marker/marker_display.hpp"

class MarkersTestFixture : public DisplayTestFixture
{
public:
  void SetUp() override
  {
    DisplayTestFixture::SetUp();
    marker_display_ = std::make_shared<rviz_default_plugins::displays::MarkerDisplay>();
  }

  void TearDown() override
  {
    marker_display_.reset();
    DisplayTestFixture::TearDown();
  }

  template<typename MarkerType>
  std::unique_ptr<MarkerType> makeMarker()
  {
    return std::make_unique<MarkerType>(
      marker_display_.get(),
      context_.get(),
      scene_manager_->getRootSceneNode()->createChildSceneNode());
  }

  std::shared_ptr<rviz_default_plugins::displays::MarkerDisplay> marker_display_;
};

visualization_msgs::msg::Marker createDefaultMessage(int32_t type)
{
  auto header = std_msgs::msg::Header();
  header.frame_id = "marker_frame";
  header.stamp = rclcpp::Clock().now();

  std::string ns = "test_ns";
  int id = 0;

  auto marker = visualization_msgs::msg::Marker();
  marker.header = header;
  marker.ns = ns;
  marker.id = id;

  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = 1;
  marker.scale.y = 0.2f;
  marker.scale.z = 0.2f;

  marker.color.a = 1.0f;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;

  marker.pose.position.x = cos(Ogre::Math::PI / 2);
  marker.pose.position.y = sin(Ogre::Math::PI / 2);
  marker.pose.position.z = 5.0f;

  marker.pose.orientation.x = 1.0f;
  marker.pose.orientation.y = 1.0f;
  marker.pose.orientation.z = 1.0f;
  marker.pose.orientation.w = 1.0f;

  marker.mesh_resource = "package://rviz_rendering_tests/test_meshes/pr2-base.dae";
  marker.mesh_use_embedded_materials = true;
  return marker;
}

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__MARKERS_TEST_FIXTURE_HPP_
