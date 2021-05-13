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

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <utility>

#include <OgreRoot.h>

#include "visualization_msgs/msg/marker.hpp"

#include "rviz_common/display.hpp"

#include "rviz_default_plugins/displays/marker/marker_common.hpp"
#include "rviz_default_plugins/displays/marker/markers/arrow_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/shape_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/line_strip_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/line_list_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/points_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/text_view_facing_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/mesh_resource_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/triangle_list_marker.hpp"

#include "../../scene_graph_introspection.hpp"
#include "marker_messages.hpp"
#include "../display_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class MarkerCommonFixture : public DisplayTestFixture
{
public:
  MarkerCommonFixture()
  {
    display_ = std::make_unique<rviz_common::Display>();

    auto scene_node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    common_ = std::make_unique<rviz_default_plugins::displays::MarkerCommon>(display_.get());
    common_->initialize(context_.get(), scene_node);
  }

  ~MarkerCommonFixture() override
  {
    common_.reset();
  }

  std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> common_;
  std::unique_ptr<rviz_common::Display> display_;
};

visualization_msgs::msg::Marker::SharedPtr createSharedPtrMessage(
  int32_t action, int32_t type, int id = 0)
{
  auto marker = createMessageWithPoints(type);
  marker.action = action;
  marker.id = id;
  return std::make_shared<visualization_msgs::msg::Marker>(marker);
}

visualization_msgs::msg::Marker::SharedPtr createSharedPtrMessage(
  int32_t action, int32_t type, const std::string & ns)
{
  auto marker = createSharedPtrMessage(action, type);
  marker->ns = ns;
  return marker;
}

visualization_msgs::msg::Marker::SharedPtr createSharedPtrMessage(
  int32_t action, const std::string & ns)
{
  return createSharedPtrMessage(action, -1, ns);
}

TEST_F(MarkerCommonFixture, processMessage_creates_correct_marker_on_add_type) {
  mockValidTransform();

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::TEXT_VIEW_FACING));

  auto text = rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode());
  ASSERT_TRUE(text);
  EXPECT_THAT(text->getCaption(), StrEq("Displaytext"));
}


TEST_F(MarkerCommonFixture, processMessage_deletes_correct_marker_on_delete_type) {
  mockValidTransform();

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      0));

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD,
      visualization_msgs::msg::Marker::POINTS,
      1));

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::DELETE,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      0));

  ASSERT_FALSE(rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode()));
  ASSERT_TRUE(rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode()));
}

TEST_F(MarkerCommonFixture, processMessage_with_scoped_deleteall_deletes_correct_markers) {
  mockValidTransform();

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      "ns_a"));

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD,
      visualization_msgs::msg::Marker::POINTS,
      "ns_b"));

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::DELETEALL,
      "ns_a"));

  ASSERT_FALSE(rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode()));
  ASSERT_TRUE(rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode()));
}

TEST_F(MarkerCommonFixture, processMessage_with_deleteall_deletes_all_markers) {
  mockValidTransform();

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      0));

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD,
      visualization_msgs::msg::Marker::POINTS,
      1));

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::DELETEALL,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      3));

  ASSERT_FALSE(rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode()));
  ASSERT_FALSE(rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode()));
}

TEST_F(MarkerCommonFixture, proccesMessage_add_all_markers_correctly) {
  mockValidTransform();

  auto marker = createSharedPtrMessage(
    visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  common_->processMessage(marker);
  ASSERT_TRUE(rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode()));

  common_->deleteAllMarkers();

  marker->type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  common_->processMessage(marker);
  ASSERT_TRUE(
    rviz_default_plugins::findEntityByMeshName(
      scene_manager_->getRootSceneNode(), marker->mesh_resource));

  common_->deleteAllMarkers();

  marker->type = visualization_msgs::msg::Marker::ARROW;
  common_->processMessage(marker);
  ASSERT_TRUE(rviz_default_plugins::findOneArrow(scene_manager_->getRootSceneNode()));

  common_->deleteAllMarkers();

  marker->type = visualization_msgs::msg::Marker::LINE_LIST;
  common_->processMessage(marker);
  ASSERT_TRUE(rviz_default_plugins::findOneBillboardChain(scene_manager_->getRootSceneNode()));

  common_->deleteAllMarkers();

  marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
  common_->processMessage(marker);
  ASSERT_TRUE(rviz_default_plugins::findOneBillboardChain(scene_manager_->getRootSceneNode()));

  common_->deleteAllMarkers();

  marker->type = visualization_msgs::msg::Marker::CYLINDER;
  common_->processMessage(marker);
  ASSERT_TRUE(
    rviz_default_plugins::findEntityByMeshName(
      scene_manager_->getRootSceneNode(), "rviz_cylinder.mesh"));

  common_->deleteAllMarkers();

  geometry_msgs::msg::Point point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  marker->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker->points.push_back(point);
  common_->processMessage(marker);
  ASSERT_TRUE(rviz_default_plugins::findOneManualObject(scene_manager_->getRootSceneNode()));

  common_->deleteAllMarkers();
}

TEST_F(MarkerCommonFixture, processMessage_adds_two_markers_of_same_type_if_ids_are_different) {
  mockValidTransform();

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD,
      visualization_msgs::msg::Marker::ARROW,
      0));

  EXPECT_THAT(rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode()), SizeIs(1));

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD,
      visualization_msgs::msg::Marker::ARROW,
      1));

  EXPECT_THAT(rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode()), SizeIs(2));
}

TEST_F(
  MarkerCommonFixture,
  processMessage_adds_two_markers_of_same_type_if_namespaces_are_different) {
  mockValidTransform();

  auto marker = createSharedPtrMessage(
    visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::ARROW);

  common_->processMessage(marker);

  EXPECT_THAT(rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode()), SizeIs(1));

  marker->ns = "new_ns";
  common_->processMessage(marker);

  EXPECT_THAT(rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode()), SizeIs(2));
}

TEST_F(MarkerCommonFixture, processMessage_does_not_create_new_marker_if_id_already_exists) {
  mockValidTransform();

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD,
      visualization_msgs::msg::Marker::ARROW,
      0));

  EXPECT_THAT(rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode()), SizeIs(1));

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD,
      visualization_msgs::msg::Marker::ARROW,
      0));

  EXPECT_THAT(rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode()), SizeIs(1));
}

TEST_F(MarkerCommonFixture, processMessage_updates_modified_marker) {
  mockValidTransform();

  auto marker = createSharedPtrMessage(
    visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  common_->processMessage(marker);

  auto before_update = rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode());
  ASSERT_TRUE(before_update);
  EXPECT_THAT(before_update->getCaption(), StrEq(marker->text));

  marker->text = "New text";
  common_->processMessage(marker);

  auto after_update = rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode());
  ASSERT_TRUE(after_update);
  EXPECT_THAT(after_update->getCaption(), StrEq("New text"));
}

TEST_F(MarkerCommonFixture, processMessage_using_modify_works_like_add) {
  mockValidTransform();

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD,
      visualization_msgs::msg::Marker::ARROW,
      0));

  EXPECT_THAT(rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode()), SizeIs(1));

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::MODIFY,
      visualization_msgs::msg::Marker::ARROW,
      0));

  EXPECT_THAT(rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode()), SizeIs(1));
}

TEST_F(MarkerCommonFixture, update_retransforms_frame_locked_messages) {
  auto marker = createSharedPtrMessage(
    visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::POINTS);
  marker->frame_locked = true;

  Ogre::Vector3 starting_position(1, 2, 3);
  Ogre::Vector3 next_position(3, 4, 5);
  Ogre::Quaternion starting_orientation(0, 0, 1, 0);
  Ogre::Quaternion next_orientation(1, 0, 0, 0);
  EXPECT_CALL(
    *frame_manager_,
    transform(_, _, _, _, _))  // NOLINT
  .WillOnce(
    DoAll(
      SetArgReferee<3>(starting_position),
      SetArgReferee<4>(starting_orientation),
      Return(true)
  ))
  .WillOnce(
    DoAll(
      SetArgReferee<3>(next_position),
      SetArgReferee<4>(next_orientation),
      Return(true)));

  common_->processMessage(marker);

  auto pointCloud = rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode());
  ASSERT_TRUE(pointCloud);
  EXPECT_THAT(pointCloud->getParentSceneNode()->getPosition(), Vector3Eq(starting_position));
  EXPECT_THAT(
    pointCloud->getParentSceneNode()->getOrientation(),
    QuaternionEq(starting_orientation));

  common_->update(0, 0);

  EXPECT_THAT(pointCloud->getParentSceneNode()->getPosition(), Vector3Eq(next_position));
  EXPECT_THAT(
    pointCloud->getParentSceneNode()->getOrientation(),
    QuaternionEq(next_orientation));
}

TEST_F(MarkerCommonFixture, update_does_not_retransform_normal_messages) {
  auto marker = createSharedPtrMessage(
    visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::POINTS);
  marker->frame_locked = false;

  Ogre::Vector3 starting_position(1, 2, 3);
  Ogre::Quaternion starting_orientation(0, 0, 1, 0);
  EXPECT_CALL(
    *frame_manager_,
    transform(_, _, _, _, _))  // NOLINT
  .WillOnce(
    DoAll(
      SetArgReferee<3>(starting_position),
      SetArgReferee<4>(starting_orientation),
      Return(true)
  ));  // Test will fail if this function is called repeatedly

  common_->processMessage(marker);

  auto pointCloud = rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode());
  ASSERT_TRUE(pointCloud);
  EXPECT_THAT(pointCloud->getParentSceneNode()->getPosition(), Vector3Eq(starting_position));
  EXPECT_THAT(
    pointCloud->getParentSceneNode()->getOrientation(),
    QuaternionEq(starting_orientation));

  common_->update(0, 0);

  EXPECT_THAT(pointCloud->getParentSceneNode()->getPosition(), Vector3Eq(starting_position));
  EXPECT_THAT(
    pointCloud->getParentSceneNode()->getOrientation(),
    QuaternionEq(starting_orientation));
}

TEST_F(MarkerCommonFixture, processMessage_adds_new_namespace_for_message) {
  mockValidTransform();

  ASSERT_THAT(display_->findProperty("Namespaces")->numChildren(), Eq(0));

  auto marker = createSharedPtrMessage(
    visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::POINTS);

  common_->processMessage(marker);

  ASSERT_THAT(
    display_->findProperty("Namespaces")->childAt(0)->getName().toStdString(),
    StrEq("test_ns"));
}

TEST_F(MarkerCommonFixture, processMessage_does_not_add_new_namespace_if_already_present) {
  mockValidTransform();

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::POINTS));

  ASSERT_THAT(display_->findProperty("Namespaces")->numChildren(), Eq(1));
  ASSERT_THAT(
    display_->findProperty("Namespaces")->childAt(0)->getName().toStdString(),
    StrEq("test_ns"));

  common_->processMessage(
    createSharedPtrMessage(
      visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::TEXT_VIEW_FACING));

  ASSERT_THAT(display_->findProperty("Namespaces")->numChildren(), Eq(1));
  ASSERT_THAT(
    display_->findProperty("Namespaces")->childAt(0)->getName().toStdString(),
    StrEq("test_ns"));
}

TEST_F(MarkerCommonFixture, onEnableChanged_in_namespace_removes_all_markers_in_that_namespace) {
  mockValidTransform();

  auto marker = createSharedPtrMessage(
    visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::POINTS);

  common_->processMessage(marker);
  marker->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker->ns = "new_ns";
  common_->processMessage(marker);

  EXPECT_TRUE(rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode()));
  EXPECT_TRUE(rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode()));

  auto namespace_property = dynamic_cast<rviz_default_plugins::displays::MarkerNamespace *>(
    display_->findProperty("Namespaces")->childAt(0));
  namespace_property->setValue(false);

  EXPECT_FALSE(rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode()));
  EXPECT_TRUE(rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode()));
}

TEST_F(MarkerCommonFixture, processMessage_does_not_add_message_with_disabled_namespace) {
  mockValidTransform();

  auto marker = createSharedPtrMessage(
    visualization_msgs::msg::Marker::ADD, visualization_msgs::msg::Marker::POINTS);

  // this is necessary to initialize namespace as we don't load a config
  common_->processMessage(marker);

  EXPECT_TRUE(rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode()));

  auto namespace_property = dynamic_cast<rviz_default_plugins::displays::MarkerNamespace *>(
    display_->findProperty("Namespaces")->childAt(0));
  namespace_property->setValue(false);

  marker->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  common_->processMessage(marker);

  EXPECT_FALSE(rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode()));
  EXPECT_FALSE(rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode()));
}
