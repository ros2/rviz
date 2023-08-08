/*
 * Copyright (c) 2019, Martin Idel
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

#include <gmock/gmock.h>

#include <memory>

#include <OgreRoot.h>
#include <OgreSceneNode.h>

#include "../ogre_testing_environment.hpp"
#include "../scene_graph_introspection.hpp"
#include "rviz_rendering/objects/screw_visual.hpp"

using namespace ::testing;  // NOLINT

MATCHER_P(Vector3Eq, expected, "") {
  return Ogre::Math::Abs(expected.x - arg.x) < 0.0001f &&
         Ogre::Math::Abs(expected.y - arg.y) < 0.0001f &&
         Ogre::Math::Abs(expected.z - arg.z) < 0.0001f;
}

MATCHER_P(ColorEq, expected, "") {
  return Ogre::Math::Abs(expected.a - arg.a) < 0.0001f &&
         Ogre::Math::Abs(expected.r - arg.r) < 0.0001f &&
         Ogre::Math::Abs(expected.g - arg.g) < 0.0001f &&
         Ogre::Math::Abs(expected.b - arg.b) < 0.0001f;
}

MATCHER_P(QuaterionEq, expected, "") {
  return Ogre::Math::Abs(expected.x - arg.x) < 0.0001f &&
         Ogre::Math::Abs(expected.y - arg.y) < 0.0001f &&
         Ogre::Math::Abs(expected.z - arg.z) < 0.0001f &&
         Ogre::Math::Abs(expected.w - arg.w) < 0.0001f;
}

class ScrewVisualTestFixture : public ::testing::Test
{
protected:
  void SetUp()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;
};

Ogre::SceneNode * findLinearArrow(Ogre::SceneNode * scene_node)
{
  auto arrows = rviz_rendering::findAllArrows(scene_node);
  auto billboard_line = rviz_rendering::findOneBillboardChain(scene_node);
  for (const auto & arrow : arrows) {
    if (billboard_line->getParentSceneNode()->getParent() != arrow->getParent()) {
      return arrow;
    }
  }
  return nullptr;
}

Ogre::SceneNode * findAngularArrow(Ogre::SceneNode * scene_node)
{
  auto arrows = rviz_rendering::findAllArrows(scene_node);
  auto billboard_line = rviz_rendering::findOneBillboardChain(scene_node);
  for (const auto & arrow : arrows) {
    if (billboard_line->getParentSceneNode()->getParent() == arrow->getParent()) {
      return arrow;
    }
  }
  return nullptr;
}

TEST_F(ScrewVisualTestFixture, setScrew_sets_linear_arrow_correctly) {
  auto scene_manager = Ogre::Root::getSingletonPtr()->createSceneManager();
  auto root_node = scene_manager->getRootSceneNode();

  auto screw_visual = std::make_shared<rviz_rendering::ScrewVisual>(scene_manager, root_node);
  ASSERT_NE(nullptr, screw_visual);

  std::vector<Ogre::SceneNode *> arrows = rviz_rendering::findAllArrows(root_node);
  EXPECT_THAT(arrows, SizeIs(3u));
  auto linear_arrow = findLinearArrow(root_node);
  auto angular_arrow = findAngularArrow(root_node);
  EXPECT_THAT(linear_arrow->getScale(), Vector3Eq(Ogre::Vector3(1, 1, 1)));
  EXPECT_THAT(angular_arrow->getScale(), Vector3Eq(Ogre::Vector3(1, 1, 1)));
  EXPECT_THAT(
    arrows[0]->convertWorldToLocalPosition(Ogre::Vector3(0, 0, 0)),
    Vector3Eq(Ogre::Vector3(0, 0, 0)));
  EXPECT_THAT(
    arrows[1]->convertWorldToLocalPosition(Ogre::Vector3(0, 0, 0)),
    Vector3Eq(Ogre::Vector3(0, 0, 0)));
  EXPECT_THAT(
    arrows[2]->convertWorldToLocalPosition(Ogre::Vector3(0, 0, 0)),
    Vector3Eq(Ogre::Vector3(0, 0, 0)));

  EXPECT_THAT(
    arrows[0]->convertWorldToLocalOrientation(Ogre::Quaternion()),
    QuaterionEq(Ogre::Quaternion(0.707107f, 0.707107f, 0.0f, 0.0f)));
  EXPECT_THAT(
    arrows[1]->convertWorldToLocalOrientation(Ogre::Quaternion()),
    QuaterionEq(Ogre::Quaternion(0.707107f, 0.707107f, 0.0f, 0.0f)));
  EXPECT_THAT(
    arrows[2]->convertWorldToLocalOrientation(Ogre::Quaternion()),
    QuaterionEq(Ogre::Quaternion(0.707107f, 0.707107f, 0.0f, 0.0f)));

  screw_visual->setLinearScale(1);
  screw_visual->setAngularScale(2);
  screw_visual->setScrew(Ogre::Vector3(1, 1, 1), Ogre::Vector3(1, 1, 1));
  linear_arrow = findLinearArrow(root_node);
  EXPECT_THAT(linear_arrow->getScale(), Vector3Eq(Ogre::Vector3(0.0f, 1.73205f, 0.0f)));

  angular_arrow = findAngularArrow(root_node);
  EXPECT_THAT(angular_arrow->getScale(), Vector3Eq(Ogre::Vector3(0.0f, 3.4641f, 0.0f)));
}
