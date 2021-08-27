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

#include <array>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <Ogre.h>

#include "rviz_rendering/objects/covariance_visual.hpp"
#include "../matcher.hpp"
#include "../ogre_testing_environment.hpp"
#include "../scene_graph_introspection.hpp"

using namespace ::testing;  // NOLINT
using rviz_rendering::findAllCones;
using rviz_rendering::findAllCylinders;
using rviz_rendering::findAllSpheres;

class CovarianceVisualTestFixture : public ::testing::Test
{
protected:
  void SetUp()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  std::tuple<Ogre::Vector3, Ogre::Real>
  setupCovarianceVisual(std::array<double, 36> covariances)
  {
    Ogre::Vector3 position(7.0, 3.0, 0.0);
    Ogre::Real offset = 2.0f;
    auto scene_manager = Ogre::Root::getSingletonPtr()->createSceneManager();
    auto root_node = scene_manager->getRootSceneNode();

    covariance_visual_ = std::make_unique<rviz_rendering::CovarianceVisual>(
      scene_manager, root_node, false, true, .1f, .1f, offset);
    covariance_visual_->setPosition(position);
    covariance_visual_->setCovariance(Ogre::Quaternion(1.57f, 0.0, 0.0, 1.0f), covariances);

    covariance_visual_node_ = dynamic_cast<Ogre::SceneNode *>(root_node->getChild(0));

    return std::make_tuple(position, offset);
  }

  std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;

  std::unique_ptr<rviz_rendering::CovarianceVisual> covariance_visual_;
  Ogre::SceneNode * covariance_visual_node_;
};

// *INDENT-OFF* - uncrustify cannot deal with layout of matrices here
std::array<double, 36> getCovariances3D()
{
  return std::array<double, 36>{{
    0.75, 0.04, 0.1,  0,    0,    0,
    0.04, 0.7,  0.4,  0,    0,    0,
    0.1,  0.4,  0.5,  0,    0,    0,
    0,    0,    0,    0.8,  0.25, 0.06,
    0,    0,    0,    0.25, 0.3,  0.22,
    0,    0,    0,    0.06, 0.22, 0.6
  }};
}

std::array<double, 36> getCovariances2D()
{
  return std::array<double, 36>{{
    0.2,  0.04, 0, 0, 0, 0,
    0.04, 0.7,  0, 0, 0, 0,
    0,    0,    0, 0, 0, 0,
    0,    0,    0, 0, 0, 0,
    0,    0,    0, 0, 0, 0,
    0,    0,    0, 0, 0, 0.6
  }};
}
// *INDENT-ON*


void assertCovarianceOrientationOffsetIsPresent(
  std::vector<Ogre::Entity *> entities, Ogre::Vector3 direction, Ogre::Real length)
{
  bool found = false;
  for (auto entity : entities) {
    if (Matches(Vector3Eq(direction))(
        entity->getParentSceneNode()->getParentSceneNode()
        ->getParentSceneNode()->getPosition()) &&
      Matches(Vector3Eq(Ogre::Vector3(length, length, length)))(
        entity->getParentSceneNode()
        ->getParentSceneNode()->getParentSceneNode()->getParentSceneNode()->getScale()))
    {
      found = true;
    }
  }
  ASSERT_TRUE(found);
}

TEST_F(CovarianceVisualTestFixture, covariance_visual_3D)
{
  Ogre::Vector3 position;
  Ogre::Real offset;
  std::tie(position, offset) = setupCovarianceVisual(getCovariances3D());

  EXPECT_THAT(covariance_visual_node_->getPosition(), Vector3Eq(position));
  auto all_spheres = findAllSpheres(covariance_visual_node_);
  ASSERT_THAT(all_spheres, SizeIs(1));
  EXPECT_THAT(
    all_spheres[0]->getParentSceneNode()->getPosition(),
    Vector3Eq(Ogre::Vector3(0.0, 0.0, 0.0)));
  EXPECT_TRUE(all_spheres[0]->isVisible());

  auto all_orientation_uncertainties = findAllCylinders(covariance_visual_node_);
  ASSERT_THAT(all_orientation_uncertainties, SizeIs(3));
  assertCovarianceOrientationOffsetIsPresent(
    all_orientation_uncertainties,
    Ogre::Vector3::UNIT_Y, offset);
  EXPECT_TRUE(all_orientation_uncertainties[0]->isVisible());
  assertCovarianceOrientationOffsetIsPresent(
    all_orientation_uncertainties,
    Ogre::Vector3::UNIT_X, offset);
  EXPECT_TRUE(all_orientation_uncertainties[1]->isVisible());
  assertCovarianceOrientationOffsetIsPresent(
    all_orientation_uncertainties,
    Ogre::Vector3::UNIT_Z, offset);
  EXPECT_TRUE(all_orientation_uncertainties[2]->isVisible());

  all_orientation_uncertainties = findAllCones(covariance_visual_node_);
  ASSERT_THAT(all_orientation_uncertainties, SizeIs(1));
  EXPECT_FALSE(all_orientation_uncertainties[0]->isVisible());
}

TEST_F(CovarianceVisualTestFixture, covariance_visual_2D)
{
  Ogre::Vector3 position;
  Ogre::Real offset;
  std::tie(position, offset) = setupCovarianceVisual(getCovariances2D());

  EXPECT_THAT(covariance_visual_node_->getPosition(), Vector3Eq(position));
  auto all_spheres = findAllSpheres(covariance_visual_node_);
  ASSERT_THAT(all_spheres, SizeIs(1));
  EXPECT_THAT(
    all_spheres[0]->getParentSceneNode()->getPosition(),
    Vector3Eq(Ogre::Vector3(0.0, 0.0, 0.0)));
  EXPECT_TRUE(all_spheres[0]->isVisible());

  auto all_orientation_uncertainties = findAllCones(covariance_visual_node_);
  ASSERT_THAT(all_orientation_uncertainties, SizeIs(1));
  assertCovarianceOrientationOffsetIsPresent(
    all_orientation_uncertainties, Ogre::Vector3(0.49115f, 0, 0), offset);
  EXPECT_TRUE(all_orientation_uncertainties[0]->isVisible());

  all_orientation_uncertainties = findAllCylinders(covariance_visual_node_);
  ASSERT_THAT(all_orientation_uncertainties, SizeIs(3));
  EXPECT_FALSE(all_orientation_uncertainties[0]->isVisible());
  EXPECT_FALSE(all_orientation_uncertainties[1]->isVisible());
  EXPECT_FALSE(all_orientation_uncertainties[2]->isVisible());
}
