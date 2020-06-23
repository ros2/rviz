/*
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include <OgreEntity.h>
#include <OgreMesh.h>
#include <OgreManualObject.h>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include "rviz_default_plugins/displays/point/point_stamped_display.hpp"

#include "../../scene_graph_introspection.hpp"
#include "../display_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class PointStampedTestFixture : public DisplayTestFixture
{
public:
  PointStampedTestFixture()
  {
    point_stamped_display_ = std::make_shared<rviz_default_plugins::displays::PointStampedDisplay>(
      context_.get());
  }

  std::shared_ptr<rviz_default_plugins::displays::PointStampedDisplay> point_stamped_display_;
};

std::shared_ptr<geometry_msgs::msg::PointStamped> createPointMessage(double x, double y, double z)
{
  auto message = std::make_shared<geometry_msgs::msg::PointStamped>();
  message->header = std_msgs::msg::Header();
  message->header.frame_id = "point_frame";
  message->header.stamp = rclcpp::Clock().now();

  message->point.x = x;
  message->point.y = y;
  message->point.z = z;

  return message;
}

void assertPointsPresent(std::vector<Ogre::Entity *> entities, Ogre::Vector3 position)
{
  bool found = false;
  for (auto entity : entities) {
    if (
      Matches(Vector3Eq(position))(
        entity->getParentSceneNode()->getParentSceneNode()->getPosition()))
    {
      found = true;
    }
  }
  ASSERT_TRUE(found);
}

TEST_F(PointStampedTestFixture, processMessage_adds_nothing_to_scene_if_invalid_transformation) {
  EXPECT_CALL(*frame_manager_, getTransform(_, _, _, _)).WillOnce(Return(false));  // NOLINT

  point_stamped_display_->processMessage(createPointMessage(0, 0, 0));

  auto objects = rviz_default_plugins::findAllEntitiesByMeshName(
    scene_manager_->getRootSceneNode(), "rviz_sphere.mesh");
  EXPECT_THAT(objects.size(), Eq(0u));
}

TEST_F(
  PointStampedTestFixture, processMessage_stores_no_more_messages_in_scene_than_history_allows)
{
  mockValidTransform();
  EXPECT_THAT(point_stamped_display_->childAt(4)->getNameStd(), StrEq("History Length"));
  point_stamped_display_->childAt(4)->setValue(2);

  point_stamped_display_->processMessage(createPointMessage(0, 0, 0));
  point_stamped_display_->processMessage(createPointMessage(1, 0, 0));
  point_stamped_display_->processMessage(createPointMessage(-1, 0, 0));

  auto objects = rviz_default_plugins::findAllEntitiesByMeshName(
    scene_manager_->getRootSceneNode(), "rviz_sphere.mesh");
  EXPECT_THAT(objects.size(), Eq(2u));
  assertPointsPresent(objects, Ogre::Vector3(1, 0, 0));
  assertPointsPresent(objects, Ogre::Vector3(-1, 0, 0));
}
