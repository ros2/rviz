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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>

#include <OgreRoot.h>

#include "visualization_msgs/msg/marker.hpp"

#include "test/rviz_default_plugins/mock_display_context.hpp"
#include "../../../../src/rviz_default_plugins/displays/marker/marker_display.hpp"
#include "src/rviz_default_plugins/displays/marker/markers/marker_factory.hpp"
#include "src/rviz_default_plugins/displays/marker/markers/arrow_marker.hpp"
#include "src/rviz_default_plugins/displays/marker/markers/shape_marker.hpp"
#include "src/rviz_default_plugins/displays/marker/markers/line_strip_marker.hpp"
#include "src/rviz_default_plugins/displays/marker/markers/line_list_marker.hpp"
#include "src/rviz_default_plugins/displays/marker/markers/points_marker.hpp"
#include \
  "src/rviz_default_plugins/displays/marker/markers/text_view_facing_marker.hpp"
#include "src/rviz_default_plugins/displays/marker/markers/mesh_resource_marker.hpp"
#include "src/rviz_default_plugins/displays/marker/markers/triangle_list_marker.hpp"

#include "test/rviz_default_plugins/displays/marker/markers/markers_test_fixture.hpp"
#include "../../scene_graph_introspection.hpp"

using namespace ::testing;  // NOLINT

class MarkerDisplayFixture : public MarkersTestFixture
{
public:
  void SetUp() override
  {
    MarkersTestFixture::SetUp();
    context_ = std::make_shared<MockDisplayContext>();
    EXPECT_CALL(*context_, getSceneManager()).WillRepeatedly(Return(scene_manager_));

    factory_ = std::make_unique<rviz_default_plugins::displays::markers::MarkerFactory>();
    factory_->initialize(nullptr, context_.get(),
      scene_manager_->getRootSceneNode()->createChildSceneNode());

    display_ = std::make_unique<rviz_default_plugins::displays::MarkerDisplay>(std::move(factory_));
  }

  std::unique_ptr<rviz_default_plugins::displays::markers::MarkerFactory> factory_;
  std::unique_ptr<rviz_default_plugins::displays::MarkerDisplay> display_;
};

visualization_msgs::msg::Marker::ConstSharedPtr createConstSharedPtrMessage(
  int32_t action, int32_t type)
{
  std::string ns = "test_ns";
  int id = 0;

  auto marker = std::shared_ptr<visualization_msgs::msg::Marker>();
  marker->header = std_msgs::msg::Header();
  marker->header.frame_id = "marker_frame";
  marker->header.stamp = rclcpp::Clock().now();
  marker->ns = ns;
  marker->id = id;

  marker->type = type;
  marker->action = action;

  marker->scale.x = 1;
  marker->scale.y = 0.2f;
  marker->scale.z = 0.2f;

  marker->color.a = 1.0f;
  marker->color.r = 0.0f;
  marker->color.g = 1.0f;
  marker->color.b = 1.0f;

  marker->pose.position.x = cos(Ogre::Math::PI / 2);
  marker->pose.position.y = sin(Ogre::Math::PI / 2);
  marker->pose.position.z = 5.0f;

  marker->pose.orientation.x = 1.0f;
  marker->pose.orientation.y = 1.0f;
  marker->pose.orientation.z = 1.0f;
  marker->pose.orientation.w = 1.0f;

  marker->text = "Displaytext";

  marker->mesh_resource = "package://rviz_rendering_tests/test_meshes/pr2-base.dae";
  marker->mesh_use_embedded_materials = true;
  return marker;
}

TEST_F(MarkerDisplayFixture, processMessage_creates_correct_marker_on_add_type) {
  mockValidTransform();

  display_->processMessage(createConstSharedPtrMessage(
    visualization_msgs::msg::Marker::ADD,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING));

  auto text = rviz_default_plugins::findMovableText(scene_manager_->getRootSceneNode());
  ASSERT_TRUE(text);
  EXPECT_EQ("Displaytext", text->getCaption());
}


TEST_F(MarkerDisplayFixture, processMessage_deletes_correct_marker_on_delete_type) {
  mockValidTransform();

  display_->processMessage(createConstSharedPtrMessage(
    visualization_msgs::msg::Marker::ADD,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING));

  display_->processMessage(createConstSharedPtrMessage(
    visualization_msgs::msg::Marker::DELETE,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING));

  auto text = rviz_default_plugins::findMovableText(scene_manager_->getRootSceneNode());
  ASSERT_TRUE(text);
}
