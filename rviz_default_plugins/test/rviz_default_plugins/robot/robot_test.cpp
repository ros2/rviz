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

#include <cmath>
#include <memory>
#include <string>

#include <QApplication>  // NOLINT

#include <OgreRoot.h>
#include <OgreSceneNode.h>

#include "resource_retriever/retriever.hpp"

#include "rviz_common/properties/property.hpp"
#include "rviz_rendering/material_manager.hpp"

#include "rviz_default_plugins/robot/robot.hpp"
#include "rviz_default_plugins/robot/robot_joint.hpp"
#include "rviz_default_plugins/robot/robot_link.hpp"

#include "../ogre_testing_environment.hpp"
#include "./mock_link_updater.hpp"
#include "../mock_display_context.hpp"
#include "../mock_handler_manager.hpp"
#include "../mock_selection_manager.hpp"
#include "../scene_graph_introspection.hpp"

using namespace ::testing;  // NOLINT
using namespace rviz_default_plugins::robot;  // NOLINT

static const char * const details_property_name = "Details";

class RobotTestFixture : public Test
{
public:
  static void SetUpTestCase()
  {
    testing_environment_ = std::make_shared<rviz_default_plugins::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();

    scene_manager_ = Ogre::Root::getSingletonPtr()->createSceneManager();

    rviz_rendering::MaterialManager::createDefaultColorMaterials();
  }

  RobotTestFixture()
  {
    selection_manager_ = std::make_shared<NiceMock<MockSelectionManager>>();
    handle_manager_ = std::make_unique<NiceMock<MockHandlerManager>>();

    context_ = std::make_shared<NiceMock<MockDisplayContext>>();
    EXPECT_CALL(*context_, getSceneManager()).WillRepeatedly(Return(scene_manager_));
    EXPECT_CALL(*context_, getSelectionManager()).WillRepeatedly(Return(selection_manager_));
    EXPECT_CALL(*context_, getHandlerManager()).WillRepeatedly(Return(handle_manager_));

    resource_retriever::Retriever retriever;
    auto file = retriever.get("package://rviz_default_plugins/test_meshes/test.urdf");
    std::string string(reinterpret_cast<char *>(file.data.get()), file.size);
    urdf_model_.initString(string);

    parent_ = std::make_shared<rviz_common::properties::Property>();
    robot_ = std::make_shared<Robot>(
      scene_manager_->getRootSceneNode(), context_.get(), "robot", parent_.get());
  }

  static void TearDownTestCase()
  {
    Ogre::Root::getSingletonPtr()->destroySceneManager(scene_manager_);
  }

  static std::shared_ptr<rviz_default_plugins::OgreTestingEnvironment> testing_environment_;
  static Ogre::SceneManager * scene_manager_;

  urdf::Model urdf_model_;
  std::shared_ptr<MockHandlerManager> handle_manager_;
  std::shared_ptr<MockDisplayContext> context_;
  std::shared_ptr<MockSelectionManager> selection_manager_;
  std::shared_ptr<rviz_common::properties::Property> parent_;

  std::shared_ptr<Robot> robot_;
};

Ogre::SceneManager * RobotTestFixture::scene_manager_ = nullptr;
std::shared_ptr<rviz_default_plugins::OgreTestingEnvironment>
RobotTestFixture::testing_environment_ = nullptr;

TEST_F(RobotTestFixture, load_gets_the_correct_link_and_joint_count) {
  robot_->load(urdf_model_);

  EXPECT_THAT(robot_->getLinks(), SizeIs(4));
  EXPECT_THAT(robot_->getJoints(), SizeIs(3));
}

TEST_F(RobotTestFixture, load_creates_links_and_sets_the_scene_node_positions) {
  robot_->load(urdf_model_);

  auto link1 = robot_->getLink("test_robot_link");
  EXPECT_THAT(link1->getVisualNode()->getChild(0)->getPosition(), Vector3Eq(Ogre::Vector3::ZERO));
  EXPECT_THAT(link1->getVisualNode()->getChild(0)->getScale(), Vector3Eq(Ogre::Vector3(1, 1, 1)));
  EXPECT_THAT(
    link1->getCollisionNode()->getChild(0)->getPosition(),
    Vector3Eq(Ogre::Vector3::ZERO));
  EXPECT_THAT(
    link1->getCollisionNode()->getChild(0)->getScale(),
    Vector3Eq(Ogre::Vector3(1, 1, 1)));

  auto link3 = robot_->getLink("test_robot_link_head");
  EXPECT_THAT(
    link3->getVisualNode()->getChild(0)->getPosition(),
    Vector3Eq(Ogre::Vector3(0, 0, 2)));
  EXPECT_THAT(link3->getVisualNode()->getChild(0)->getScale(), Vector3Eq(Ogre::Vector3(2, 2, 2)));
  EXPECT_THAT(link3->getCollisionNode()->numChildren(), Eq(0u));
}

TEST_F(RobotTestFixture, load_sets_joint_parents_and_children) {
  robot_->load(urdf_model_);

  EXPECT_THAT(robot_->getJoint("test_robot_fixed1")->getParentLinkName(), StrEq("test_robot_link"));
  EXPECT_THAT(
    robot_->getJoint("test_robot_fixed1")->getChildLinkName(),
    StrEq("test_robot_link_right_arm"));

  EXPECT_THAT(robot_->getJoint("test_robot_fixed2")->getParentLinkName(), StrEq("test_robot_link"));
  EXPECT_THAT(
    robot_->getJoint("test_robot_fixed2")->getChildLinkName(),
    StrEq("test_robot_link_left_arm"));

  EXPECT_THAT(
    robot_->getJoint("test_robot_continuous")->getParentLinkName(),
    StrEq("test_robot_link"));
  EXPECT_THAT(
    robot_->getJoint("test_robot_continuous")->getChildLinkName(),
    StrEq("test_robot_link_head"));
}

TEST_F(RobotTestFixture, load_creates_the_link_property_list) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  EXPECT_THAT(prop->getNameStd(), StrEq("Links"));

  // The first 5 children are fixed, some are hidden
  EXPECT_THAT(prop->childAt(5)->getNameStd(), StrEq("test_robot_link"));
  EXPECT_THAT(prop->childAt(6)->getNameStd(), StrEq("test_robot_link_head"));
}

TEST_F(RobotTestFixture, changedLinkTreeStyle_hides_joint_properties_in_link_list_mode) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  EXPECT_THAT(prop->getNameStd(), StrEq("Links"));

  EXPECT_THAT(prop->childAt(2)->getNameStd(), StrEq("Expand Link Details"));
  EXPECT_FALSE(prop->childAt(2)->getHidden());

  EXPECT_THAT(prop->childAt(3)->getNameStd(), StrEq("Expand Joint Details"));
  EXPECT_TRUE(prop->childAt(3)->getHidden());
}

TEST_F(RobotTestFixture, changedLinkTreeStyle_hides_link_properties_in_joint_list_mode) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  prop->childAt(0)->setValue("Joints in Alphabetic Order");

  EXPECT_THAT(prop->getNameStd(), StrEq("Joints"));

  EXPECT_THAT(prop->childAt(2)->getNameStd(), StrEq("Expand Link Details"));
  EXPECT_TRUE(prop->childAt(2)->getHidden());

  EXPECT_THAT(prop->childAt(3)->getNameStd(), StrEq("Expand Joint Details"));
  EXPECT_FALSE(prop->childAt(3)->getHidden());
}

TEST_F(RobotTestFixture, changedLinkTreeStyle_reorders_the_properties_into_a_joint_list) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  prop->childAt(0)->setValue("Joints in Alphabetic Order");

  EXPECT_THAT(prop->getNameStd(), StrEq("Joints"));

  // The first 5 children are fixed, some are hidden
  EXPECT_THAT(prop->childAt(5)->getNameStd(), StrEq("test_robot_continuous"));
  EXPECT_THAT(prop->childAt(6)->getNameStd(), StrEq("test_robot_fixed1"));
}

TEST_F(RobotTestFixture, changedLinkTreeStyle_reorders_the_properties_into_a_tree) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  prop->childAt(0)->setValue("Tree of links");

  EXPECT_THAT(prop->getNameStd(), StrEq("Link Tree"));
  EXPECT_THAT(prop->numChildren(), Eq(6));
  EXPECT_THAT(prop->childAt(5)->getNameStd(), StrEq("test_robot_link"));
  EXPECT_THAT(prop->childAt(5)->childAt(0)->getNameStd(), StrEq(details_property_name));
  EXPECT_THAT(prop->childAt(5)->childAt(1)->getNameStd(), StrEq("test_robot_link_head"));
}

TEST_F(RobotTestFixture, clear_removes_all_joints_and_links) {
  robot_->load(urdf_model_);

  robot_->clear();

  EXPECT_THAT(robot_->getLinks(), SizeIs(0));
  EXPECT_THAT(robot_->getJoints(), SizeIs(0));
}

TEST_F(RobotTestFixture, update_sets_position_and_orientation_in_links_and_joints) {
  robot_->load(urdf_model_);

  NiceMock<MockLinkUpdater> link_updater;
  EXPECT_CALL(link_updater, getLinkTransforms(_, _, _, _, _))
  .Times(3)
  .WillRepeatedly(Return(false));

  Ogre::Vector3 visual_position(4, 4, 4);
  Ogre::Quaternion visual_orientation(0, 1, 0, 0);
  Ogre::Vector3 collision_position(0, 0, 0);
  Ogre::Quaternion collision_orientation(0, 0, 0, 1);

  EXPECT_CALL(link_updater, getLinkTransforms(StrEq("test_robot_link"), _, _, _, _))
  .WillOnce(
    DoAll(
      SetArgReferee<1>(visual_position),
      SetArgReferee<2>(visual_orientation),
      SetArgReferee<3>(collision_position),
      SetArgReferee<4>(collision_orientation),
      Return(true)
  ));

  robot_->update(link_updater);

  auto link1 = robot_->getLink("test_robot_link");
  EXPECT_THAT(link1->getVisualNode()->getPosition(), Vector3Eq(visual_position));
  EXPECT_THAT(link1->getVisualNode()->getOrientation(), QuaternionEq(visual_orientation));
  EXPECT_THAT(link1->getCollisionNode()->getPosition(), Vector3Eq(collision_position));
  EXPECT_THAT(link1->getCollisionNode()->getOrientation(), QuaternionEq(collision_orientation));

  auto joint1 = robot_->getJoint("test_robot_fixed1");
  EXPECT_THAT(joint1->getPosition(), Vector3Eq(visual_position));
  EXPECT_THAT(joint1->getOrientation(), QuaternionEq(visual_orientation));
}

TEST_F(RobotTestFixture, link_descriptions_show_correct_hierarchy) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  prop->childAt(0)->setValue("Links in Alphabetic Order");

  EXPECT_THAT(prop->numChildren(), Eq(9));
  EXPECT_THAT(prop->childAt(0)->getNameStd(), StrEq("Link Tree Style"));
  EXPECT_THAT(prop->childAt(5)->getNameStd(), StrEq("test_robot_link"));
  EXPECT_THAT(
    prop->childAt(5)->getDescription().toStdString(),
    StrEq(
      "Root Link <b>test_robot_link</b> has 3 child joints: "
      "<b>test_robot_continuous</b>, <b>test_robot_fixed1</b>, <b>test_robot_fixed2</b>.  "
      "Check/uncheck to show/hide this link in the display."
    )
  );
}

TEST_F(RobotTestFixture, joint_descriptions_show_correct_hierarchy) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  prop->childAt(0)->setValue("Joints in Alphabetic Order");

  EXPECT_THAT(prop->numChildren(), Eq(8));
  EXPECT_THAT(prop->childAt(0)->getNameStd(), StrEq("Link Tree Style"));
  EXPECT_THAT(prop->childAt(5)->getNameStd(), StrEq("test_robot_continuous"));
  EXPECT_THAT(
    prop->childAt(5)->getDescription().toStdString(),
    StrEq(
      "Joint <b>test_robot_continuous</b> with parent link <b>test_robot_link</b> "
      "and child link <b>test_robot_link_head</b>.  "
      "Check/uncheck to show/hide this joint's child link."
    )
  );
}

TEST_F(RobotTestFixture, robot_model_link_contains_right_number_meshes) {
  robot_->load(urdf_model_);

  auto body_link = robot_->getLink("test_robot_link");
  auto right_arm_link = robot_->getLink("test_robot_link_right_arm");

  EXPECT_THAT(body_link->getCollisionMeshes(), SizeIs(1));
  EXPECT_THAT(body_link->getVisualMeshes(), SizeIs(1));
  EXPECT_THAT(right_arm_link->getCollisionMeshes(), IsEmpty());
  EXPECT_THAT(right_arm_link->getVisualMeshes(), SizeIs(1));
}

TEST_F(RobotTestFixture, expand_details_in_link_description_shows_hidden_properties) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  prop->childAt(0)->setValue("Links in Alphabetic Order");

  EXPECT_THAT(prop->numChildren(), Eq(9));
  EXPECT_THAT(prop->childAt(0)->getNameStd(), StrEq("Link Tree Style"));

  auto test_robot_link_property = prop->childAt(5);
  EXPECT_FALSE(test_robot_link_property->getHidden());
  EXPECT_THAT(test_robot_link_property->getNameStd(), StrEq("test_robot_link"));
  EXPECT_THAT(test_robot_link_property->numChildren(), Eq(5));
  EXPECT_THAT(test_robot_link_property->childAt(0)->getNameStd(), StrEq("Alpha"));
  EXPECT_THAT(test_robot_link_property->childAt(3)->getNameStd(), StrEq("Position"));
}

TEST_F(RobotTestFixture, expand_details_in_joint_description_shows_hidden_properties) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  prop->childAt(0)->setValue("Joints in Alphabetic Order");

  EXPECT_THAT(prop->numChildren(), Eq(8));
  EXPECT_THAT(prop->childAt(0)->getNameStd(), StrEq("Link Tree Style"));

  auto test_robot_link_property = prop->childAt(5);
  EXPECT_FALSE(test_robot_link_property->getHidden());
  EXPECT_THAT(test_robot_link_property->getNameStd(), StrEq("test_robot_continuous"));
  EXPECT_THAT(test_robot_link_property->numChildren(), Eq(6));
  EXPECT_THAT(test_robot_link_property->childAt(0)->getNameStd(), StrEq("Show Axes"));
  EXPECT_THAT(test_robot_link_property->childAt(3)->getNameStd(), StrEq("Type"));
}

TEST_F(RobotTestFixture, changedExpandTree_shows_link_and_joint_properties_and_hides_details) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  prop->childAt(0)->setValue("Tree of links and joints");
  prop->childAt(1)->setValue(true);  // Enable "Expand Tree"

  EXPECT_THAT(prop->getNameStd(), StrEq("Link/Joint Tree"));
  EXPECT_THAT(prop->numChildren(), Eq(6));

  auto test_robot_link = prop->childAt(5);
  EXPECT_THAT(test_robot_link->getNameStd(), StrEq("test_robot_link"));

  auto test_robot_link_details = test_robot_link->childAt(0);
  EXPECT_THAT(test_robot_link_details->getNameStd(), StrEq(details_property_name));
  EXPECT_FALSE(test_robot_link_details->isExpanded());

  auto test_robot_continuous = test_robot_link->childAt(1);
  EXPECT_THAT(test_robot_continuous->getNameStd(), StrEq("test_robot_continuous"));
  EXPECT_TRUE(test_robot_continuous->isExpanded());

  auto test_robot_continuous_details = test_robot_continuous->childAt(0);
  EXPECT_THAT(test_robot_continuous_details->getNameStd(), StrEq(details_property_name));
  EXPECT_FALSE(test_robot_continuous_details->isExpanded());
}

TEST_F(RobotTestFixture, changedExpandTree_hides_link_and_joint_properties_on_deselect) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  prop->childAt(0)->setValue("Tree of links and joints");
  prop->childAt(1)->setValue(true);  // Enable "Expand Tree"
  prop->childAt(1)->setValue(false);  // Disable "Expand Tree"

  EXPECT_THAT(prop->getNameStd(), StrEq("Link/Joint Tree"));
  EXPECT_THAT(prop->numChildren(), Eq(6));

  auto test_robot_link = prop->childAt(5);
  EXPECT_THAT(test_robot_link->getNameStd(), StrEq("test_robot_link"));
  EXPECT_FALSE(test_robot_link->isExpanded());
}


TEST_F(RobotTestFixture, changedExpandLinkDetails_shows_link_details) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  prop->childAt(0)->setValue("Tree of links and joints");
  prop->childAt(1)->setValue(true);  // Enable "Expand Tree"
  prop->childAt(2)->setValue(true);  // Enable "Expand Link Details"

  EXPECT_THAT(prop->getNameStd(), StrEq("Link/Joint Tree"));
  EXPECT_THAT(prop->numChildren(), Eq(6));

  auto test_robot_link = prop->childAt(5);
  EXPECT_THAT(test_robot_link->getNameStd(), StrEq("test_robot_link"));

  auto test_robot_link_details = test_robot_link->childAt(0);
  EXPECT_THAT(test_robot_link_details->getNameStd(), StrEq(details_property_name));
  EXPECT_TRUE(test_robot_link_details->isExpanded());

  auto test_robot_continuous = test_robot_link->childAt(1);
  EXPECT_THAT(test_robot_continuous->getNameStd(), StrEq("test_robot_continuous"));
  EXPECT_TRUE(test_robot_continuous->isExpanded());

  auto test_robot_continuous_details = test_robot_continuous->childAt(0);
  EXPECT_THAT(test_robot_continuous_details->getNameStd(), StrEq(details_property_name));
  EXPECT_FALSE(test_robot_continuous_details->isExpanded());
}

TEST_F(RobotTestFixture, changedExpandJointDetails_shows_joint_details) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  prop->childAt(0)->setValue("Tree of links and joints");
  prop->childAt(1)->setValue(true);  // Enable "Expand Tree"
  prop->childAt(3)->setValue(true);  // Enable "Expand Joint Details"

  EXPECT_THAT(prop->getNameStd(), StrEq("Link/Joint Tree"));
  EXPECT_THAT(prop->numChildren(), Eq(6));

  auto test_robot_link = prop->childAt(5);
  EXPECT_THAT(test_robot_link->getNameStd(), StrEq("test_robot_link"));

  auto test_robot_link_details = test_robot_link->childAt(0);
  EXPECT_THAT(test_robot_link_details->getNameStd(), StrEq(details_property_name));
  EXPECT_FALSE(test_robot_link_details->isExpanded());

  auto test_robot_continuous_details = test_robot_link->childAt(1)->childAt(0);
  EXPECT_THAT(test_robot_continuous_details->getNameStd(), StrEq(details_property_name));
  EXPECT_TRUE(test_robot_continuous_details->isExpanded());
}

TEST_F(RobotTestFixture, changedEnableAllLinks_toggles_all_links) {
  robot_->load(urdf_model_);

  auto prop = robot_->getLinkTreeProperty();
  EXPECT_THAT(prop->getNameStd(), StrEq("Links"));

  auto all_links_enabled = prop->childAt(4);
  EXPECT_THAT(all_links_enabled->getNameStd(), StrEq("All Links Enabled"));
  EXPECT_THAT(prop->numChildren(), Eq(9));

  all_links_enabled->setValue(true);

  EXPECT_TRUE(prop->childAt(5)->getValue().toBool());
  EXPECT_TRUE(prop->childAt(6)->getValue().toBool());
  EXPECT_TRUE(prop->childAt(7)->getValue().toBool());
  EXPECT_TRUE(prop->childAt(8)->getValue().toBool());

  all_links_enabled->setValue(false);

  EXPECT_FALSE(prop->childAt(5)->getValue().toBool());
  EXPECT_FALSE(prop->childAt(6)->getValue().toBool());
  EXPECT_FALSE(prop->childAt(7)->getValue().toBool());
  EXPECT_FALSE(prop->childAt(8)->getValue().toBool());
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
