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

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4996)
#include <OgreEntity.h>
#pragma warning(pop)
#else
#include <OgreEntity.h>
#endif

#include "../../../scene_graph_introspection.hpp"

#include "../../../../../src/rviz_default_plugins/displays/marker/markers/points_marker.hpp"

#include "markers_test_fixture.hpp"

class PointsMarkerFixture : public MarkersTestFixture
{
public:
  void SetUp() override
  {
    MarkersTestFixture::SetUp();
    marker_ = makeMarker<rviz_default_plugins::displays::markers::PointsMarker>();
  }

  void TearDown() override
  {
    marker_.reset(nullptr);
    MarkersTestFixture::TearDown();
  }

  std::unique_ptr<rviz_default_plugins::displays::markers::PointsMarker> marker_;
};

using namespace ::testing;  // NOLINT

visualization_msgs::msg::Marker createMessageWithPoints(int32_t type)
{
  geometry_msgs::msg::Point first_point;
  geometry_msgs::msg::Point second_point;

  first_point.x = 2;
  first_point.y = 0;
  first_point.z = 0;
  second_point.x = 1;
  second_point.y = 1;
  second_point.z = 0;

  auto marker = createDefaultMessage(type);
  marker.points.push_back(first_point);
  marker.points.push_back(second_point);

  return marker;
}

visualization_msgs::msg::Marker createMessageWithColorPerPoint(int32_t type)
{
  std_msgs::msg::ColorRGBA first_color;
  std_msgs::msg::ColorRGBA second_color;

  first_color.r = 1.0f;
  first_color.g = 0.0f;
  first_color.b = 0.5f;
  first_color.a = 0.5f;
  second_color.r = 0.5f;
  second_color.g = 0.6f;
  second_color.b = 0.0f;
  second_color.a = 0.3f;

  auto marker = createMessageWithPoints(type);
  marker.colors.push_back(first_color);
  marker.colors.push_back(second_color);

  return marker;
}

TEST_F(PointsMarkerFixture, setMessage_makes_the_scene_node_invisible_if_invalid_transform) {
  EXPECT_CALL(*frame_manager_, transform(_, _, _, _, _)).WillOnce(Return(false));  // NOLINT

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::POINTS));

  auto point_cloud = rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode());
  EXPECT_FALSE(point_cloud->isVisible());
}

TEST_F(PointsMarkerFixture, setMessage_sets_points_correctly) {
  mockValidTransform();

  marker_->setMessage(createMessageWithPoints(visualization_msgs::msg::Marker::SPHERE_LIST));

  auto point_cloud = rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode());
  float expected_bounding_radius = 2.236068f;
  EXPECT_TRUE(point_cloud->isVisible());
  EXPECT_FLOAT_EQ(expected_bounding_radius, point_cloud->getBoundingRadius());
}

TEST_F(PointsMarkerFixture, setMessage_sets_position_and_orientation_correctly) {
  mockValidTransform();

  marker_->setMessage(createMessageWithPoints(visualization_msgs::msg::Marker::CUBE_LIST));

  EXPECT_VECTOR3_EQ(Ogre::Vector3(0, 1, 0), marker_->getPosition());
  EXPECT_QUATERNION_EQ(Ogre::Quaternion(0, 0, 1, 0), marker_->getOrientation());
}

TEST_F(PointsMarkerFixture, setMessage_sets_single_color_correctly) {
  mockValidTransform();

  marker_->setMessage(createMessageWithPoints(visualization_msgs::msg::Marker::SPHERE_LIST));
  auto point_cloud = rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode());

  Ogre::ColourValue expected_color(0.0f, 1.0f, 1.0f, 1.0f);
  EXPECT_EQ(expected_color, point_cloud->getPoints()[0].color);
  EXPECT_EQ(expected_color, point_cloud->getPoints()[1].color);
}

TEST_F(PointsMarkerFixture, setMessage_sets_per_point_color_correctly) {
  mockValidTransform();

  marker_->setMessage(createMessageWithColorPerPoint(visualization_msgs::msg::Marker::POINTS));
  auto point_cloud = rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode());

  EXPECT_EQ(Ogre::ColourValue(1.0f, 0.0f, 0.5f, 0.5f), point_cloud->getPoints()[0].color);
  EXPECT_EQ(Ogre::ColourValue(0.5f, 0.6f, 0.0f, 0.3f), point_cloud->getPoints()[1].color);
}
