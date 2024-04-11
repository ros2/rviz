/*
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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

#include <gmock/gmock.h>

#include <OgreRoot.h>
#include <OgreSceneNode.h>

#include <memory>

#include "../ogre_testing_environment.hpp"
#include "../scene_graph_introspection.hpp"
#include "rviz_rendering/objects/effort_visual.hpp"

using namespace ::testing;  // NOLINT

MATCHER_P(ColorEq, expected, "") {
  return Ogre::Math::Abs(expected.a - arg.a) < 0.0001f &&
         Ogre::Math::Abs(expected.r - arg.r) < 0.0001f &&
         Ogre::Math::Abs(expected.g - arg.g) < 0.0001f &&
         Ogre::Math::Abs(expected.b - arg.b) < 0.0001f;
}

MATCHER_P(Vector3Eq, expected, "") {
  return Ogre::Math::Abs(expected.x - arg.x) < 0.0001f &&
         Ogre::Math::Abs(expected.y - arg.y) < 0.0001f &&
         Ogre::Math::Abs(expected.z - arg.z) < 0.0001f;
}

MATCHER_P(QuaterionEq, expected, "") {
  return Ogre::Math::Abs(expected.x - arg.x) < 0.0001f &&
         Ogre::Math::Abs(expected.y - arg.y) < 0.0001f &&
         Ogre::Math::Abs(expected.z - arg.z) < 0.0001f &&
         Ogre::Math::Abs(expected.w - arg.w) < 0.0001f;
}


class EffortVisualTestFixture : public ::testing::Test
{
protected:
  void SetUp()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;
};

static Ogre::SceneNode * findForceArrow(Ogre::SceneNode * scene_node)
{
  std::vector<Ogre::SceneNode *> arrows = rviz_rendering::findAllArrows(scene_node);
  Ogre::BillboardChain * billboard_line = rviz_rendering::findOneBillboardChain(scene_node);
  for (Ogre::SceneNode * arrow : arrows) {
    if (billboard_line->getParentSceneNode()->getParent() == arrow->getParent()) {
      return arrow;
    }
  }
  return nullptr;
}

TEST_F(EffortVisualTestFixture, setEffort_sets_force_arrow_correctly) {
  auto scene_manager = Ogre::Root::getSingletonPtr()->createSceneManager();
  auto root_node = scene_manager->getRootSceneNode();

  auto effort_visual = std::make_shared<rviz_rendering::EffortVisual>(
    scene_manager, root_node, 0.0f, 0.0f);
  ASSERT_NE(nullptr, effort_visual);

  Ogre::ColourValue color;
  effort_visual->getRainbowColor(0, color);

  EXPECT_THAT(color, ColorEq(Ogre::ColourValue(1, 0, 1, 1)));
  effort_visual->getRainbowColor(1, color);
  EXPECT_THAT(color, ColorEq(Ogre::ColourValue(1, 0, 0, 1)));

  effort_visual->setFramePosition("joint1", Ogre::Vector3(0, 0, 0));
  effort_visual->setFrameOrientation("joint1", Ogre::Quaternion());
  effort_visual->setEffort("joint1", 1, 10);

  std::vector<Ogre::SceneNode *> arrows = rviz_rendering::findAllArrows(root_node);
  EXPECT_THAT(arrows, SizeIs(1u));
  EXPECT_THAT(
    arrows[0]->convertWorldToLocalPosition(Ogre::Vector3(0, 0, 0)),
    Vector3Eq(Ogre::Vector3(0.0f, 0.0f, -0.05f)));

  EXPECT_THAT(
    arrows[0]->convertWorldToLocalOrientation(Ogre::Quaternion()),
    QuaterionEq(Ogre::Quaternion(0.5, 0.5, -0.5, -0.5)));

  Ogre::Vector3 pos1(1, 2, 3);
  effort_visual->setFramePosition("joint1", pos1);
  effort_visual->setEffort("joint1", 1, 10);
  EXPECT_THAT(
    arrows[0]->convertWorldToLocalPosition(Ogre::Vector3(0, 0, 0)),
    Vector3Eq(Ogre::Vector3(3.0f, 1.0f, -2.05f)));
}

TEST_F(EffortVisualTestFixture, setEffort_hides_force_arrow_for_larger_width_than_scale) {
  auto scene_manager = Ogre::Root::getSingletonPtr()->createSceneManager();
  auto root_node = scene_manager->getRootSceneNode();

  auto effort_visual = std::make_shared<rviz_rendering::EffortVisual>(
    scene_manager, root_node, 5.0f, 0.7f);
  ASSERT_NE(nullptr, effort_visual);

  effort_visual->setFramePosition("joint1", Ogre::Vector3(0, 0, 0));
  effort_visual->setFrameOrientation("joint1", Ogre::Quaternion());
  effort_visual->setEffort("joint1", 1, 10);

  auto arrows = rviz_rendering::findAllArrows(root_node);
  EXPECT_THAT(arrows, SizeIs(1u));
  auto force_arrow = findForceArrow(root_node);
  EXPECT_THAT(force_arrow->getScale(), Vector3Eq(Ogre::Vector3(1, 1, 1)));
}
