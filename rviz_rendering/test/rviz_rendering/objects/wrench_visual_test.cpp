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
#include <string>
#include <vector>

#include <OgreBillboardChain.h>
#include <OgreManualObject.h>
#include <OgreRoot.h>
#include <OgreSceneNode.h>

#include "../ogre_testing_environment.hpp"
#include "../scene_graph_introspection.hpp"
#include "rviz_rendering/objects/wrench_visual.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"
#include "rviz_rendering/objects/arrow.hpp"

using namespace ::testing;  // NOLINT

MATCHER_P(Vector3Eq, expected, "") {
  return Ogre::Math::Abs(expected.x - arg.x) < 0.0001f &&
         Ogre::Math::Abs(expected.y - arg.y) < 0.0001f &&
         Ogre::Math::Abs(expected.z - arg.z) < 0.0001f;
}

class WrenchVisualTestFixture : public ::testing::Test
{
protected:
  void SetUp()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;
};

Ogre::SceneNode * findForceArrow(Ogre::SceneNode * scene_node)
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

TEST_F(WrenchVisualTestFixture, setWrench_sets_force_arrow_correctly) {
  auto scene_manager = Ogre::Root::getSingletonPtr()->createSceneManager();
  auto root_node = scene_manager->getRootSceneNode();

  auto wrench_visual = std::make_shared<rviz_rendering::WrenchVisual>(scene_manager, root_node);

  wrench_visual->setForceScale(1);
  wrench_visual->setWidth(0.2f);

  wrench_visual->setWrench(Ogre::Vector3(3, 0, 0), Ogre::Vector3(0, 1, 0));

  auto arrows = rviz_rendering::findAllArrows(root_node);
  EXPECT_THAT(arrows, SizeIs(3u));
  auto force_arrow = findForceArrow(root_node);
  EXPECT_THAT(force_arrow->getScale(), Vector3Eq(Ogre::Vector3(0.2f, 3, 0.2f)));
}

TEST_F(WrenchVisualTestFixture, setWrench_hides_force_arrow_for_larger_width_than_scale) {
  auto scene_manager = Ogre::Root::getSingletonPtr()->createSceneManager();
  auto root_node = scene_manager->getRootSceneNode();

  auto wrench_visual = std::make_shared<rviz_rendering::WrenchVisual>(scene_manager, root_node);

  wrench_visual->setWrench(Ogre::Vector3(1, 0, 0), Ogre::Vector3(0, 1, 0));

  wrench_visual->setForceScale(0.2f);
  wrench_visual->setWidth(10);

  auto arrows = rviz_rendering::findAllArrows(root_node);
  EXPECT_THAT(arrows, SizeIs(3u));
  auto force_arrow = findForceArrow(root_node);
  EXPECT_THAT(force_arrow->getScale(), Vector3Eq(Ogre::Vector3(1, 1, 1)));
  auto objects = force_arrow->getAttachedObjects();
  for (const auto & object : objects) {
    EXPECT_THAT(object->isVisible(), IsFalse());
  }
}
