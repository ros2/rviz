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
#include <vector>

#include <Ogre.h>

#include "rviz_rendering/objects/covariance_visual.hpp"
#include "test/rviz_rendering/matcher.hpp"
#include "test/rviz_rendering/ogre_testing_environment.hpp"
#include "test/rviz_rendering/scene_graph_introspection.hpp"

using namespace ::testing;  // NOLINT
using rviz_rendering::findAllEntitiesByMeshName;

class CovarianceVisualTestFixture : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  static std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;
};

std::shared_ptr<rviz_rendering::OgreTestingEnvironment>
CovarianceVisualTestFixture::testing_environment_ = nullptr;

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


void assertCovarianceOrientationOffset(
  Ogre::Entity * entity, Ogre::Vector3 direction, double length)
{
  EXPECT_THAT(entity->getParentSceneNode()->getParentSceneNode()->getParentSceneNode()
    ->getPosition(), Vector3Eq(direction));
  ASSERT_THAT(entity->getParentSceneNode()->getParentSceneNode()->getParentSceneNode()
    ->getParentSceneNode()->getScale(), Vector3Eq(Ogre::Vector3(length, length, length)));
}

TEST_F(CovarianceVisualTestFixture, covariance_visual_3D)
{
  auto scene_manager = Ogre::Root::getSingletonPtr()->createSceneManager();
  auto root_node = scene_manager->getRootSceneNode();

  auto covariance_visual =
    std::make_unique<rviz_rendering::CovarianceVisual>(scene_manager, root_node, false);
  covariance_visual->setPosition(Ogre::Vector3(7.0, 3.0, 0.0));
  covariance_visual->setOrientationOffset(2.0);
  covariance_visual->setCovariance(Ogre::Quaternion(1.57, 0.0, 0.0, 1.0), getCovariances3D());

  auto covariance_visual_node = dynamic_cast<Ogre::SceneNode *>(root_node->getChild(0));

  ASSERT_THAT(covariance_visual_node->getPosition(), Vector3Eq(Ogre::Vector3(7.0, 3.0, 0.0)));
  auto all_spheres = findAllEntitiesByMeshName(covariance_visual_node, "rviz_sphere.mesh");
  ASSERT_THAT(all_spheres, SizeIs(1));
  ASSERT_THAT(all_spheres[0]->getParentSceneNode()->getPosition(),
    Vector3Eq(Ogre::Vector3(0.0, 0.0, 0.0)));
  ASSERT_TRUE(all_spheres[0]->isVisible());

  auto all_orientation_uncertainties = findAllEntitiesByMeshName(
    covariance_visual_node, "rviz_cylinder.mesh");
  ASSERT_THAT(all_orientation_uncertainties, SizeIs(3));
  assertCovarianceOrientationOffset(all_orientation_uncertainties[0], Ogre::Vector3::UNIT_Y, 2);
  ASSERT_TRUE(all_orientation_uncertainties[0]->isVisible());
  assertCovarianceOrientationOffset(all_orientation_uncertainties[1], Ogre::Vector3::UNIT_X, 2);
  ASSERT_TRUE(all_orientation_uncertainties[1]->isVisible());
  assertCovarianceOrientationOffset(all_orientation_uncertainties[2], Ogre::Vector3::UNIT_Z, 2);
  ASSERT_TRUE(all_orientation_uncertainties[2]->isVisible());

  all_orientation_uncertainties = findAllEntitiesByMeshName(
    covariance_visual_node, "rviz_cone.mesh");
  ASSERT_THAT(all_orientation_uncertainties, SizeIs(1));
  ASSERT_FALSE(all_orientation_uncertainties[0]->isVisible());
}

TEST_F(CovarianceVisualTestFixture, covariance_visual_2D)
{
  auto scene_manager = Ogre::Root::getSingletonPtr()->createSceneManager();
  auto root_node = scene_manager->getRootSceneNode();

  auto covariance_visual =
    std::make_unique<rviz_rendering::CovarianceVisual>(scene_manager, root_node, false);
  covariance_visual->setPosition(Ogre::Vector3(7.0, 3.0, 0.0));
  covariance_visual->setOrientationOffset(2.0);
  covariance_visual->setCovariance(Ogre::Quaternion(1.57, 0.0, 0.0, 1.0), getCovariances2D());

  auto covariance_visual_node = dynamic_cast<Ogre::SceneNode *>(root_node->getChild(0));

  ASSERT_THAT(covariance_visual_node->getPosition(), Vector3Eq(Ogre::Vector3(7.0, 3.0, 0.0)));
  auto all_spheres = findAllEntitiesByMeshName(covariance_visual_node, "rviz_sphere.mesh");
  ASSERT_THAT(all_spheres, SizeIs(1));
  ASSERT_THAT(all_spheres[0]->getParentSceneNode()->getPosition(),
    Vector3Eq(Ogre::Vector3(0.0, 0.0, 0.0)));
  ASSERT_TRUE(all_spheres[0]->isVisible());

  auto all_orientation_uncertainties = findAllEntitiesByMeshName(
    covariance_visual_node, "rviz_cone.mesh");
  ASSERT_THAT(all_orientation_uncertainties, SizeIs(1));
  assertCovarianceOrientationOffset(
    all_orientation_uncertainties[0], Ogre::Vector3(0.49115, 0, 0), 2);
  ASSERT_TRUE(all_orientation_uncertainties[0]->isVisible());

  all_orientation_uncertainties = findAllEntitiesByMeshName(
    covariance_visual_node, "rviz_cylinder.mesh");
  ASSERT_THAT(all_orientation_uncertainties, SizeIs(3));
  ASSERT_FALSE(all_orientation_uncertainties[0]->isVisible());
  ASSERT_FALSE(all_orientation_uncertainties[1]->isVisible());
  ASSERT_FALSE(all_orientation_uncertainties[2]->isVisible());
}
