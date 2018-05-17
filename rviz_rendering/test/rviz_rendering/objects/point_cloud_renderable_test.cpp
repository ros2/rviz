/*
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <vector>

#ifndef _WIN32
# pragma GCC diagnostic push
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wextra-semi"
#  pragma clang diagnostic ignored "-Wkeyword-macro"
# endif
# pragma GCC diagnostic ignored "-Wpedantic"
#endif

#include <OgreCamera.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_rendering/objects/point_cloud_renderable.hpp"
#include "test/rviz_rendering/ogre_testing_environment.hpp"

using namespace ::testing;  // NOLINT

class PointCloudRenderableTestFixture : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  PointCloudRenderableTestFixture()
  {
    cloud_ = std::make_shared<rviz_rendering::PointCloud>();
    auto points = std::vector<rviz_rendering::PointCloud::Point>(
      {{Ogre::Vector3(1, 1, 1), Ogre::ColourValue()}});
    cloud_->addPoints(points.begin(), points.end());
    renderable_ = cloud_->getRenderables().front();
  }

  static std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;
  std::shared_ptr<rviz_rendering::PointCloud> cloud_;
  rviz_rendering::PointCloudRenderablePtr renderable_;
};

std::shared_ptr<rviz_rendering::OgreTestingEnvironment>
PointCloudRenderableTestFixture::testing_environment_ = nullptr;

TEST_F(PointCloudRenderableTestFixture, getBoundingRadius_returns_radius_from_coordinate_origin) {
  auto boundingBox = Ogre::AxisAlignedBox(Ogre::Vector3(-1, -1, -1), Ogre::Vector3(2, 2, 0));
  renderable_->setBoundingBox(boundingBox);

  ASSERT_THAT(renderable_->getBoundingRadius(), FloatEq(Ogre::Math::Sqrt(8)));

  boundingBox.setMinimum(-2, -2, -2);
  renderable_->setBoundingBox(boundingBox);

  ASSERT_THAT(renderable_->getBoundingRadius(), FloatEq(Ogre::Math::Sqrt(12)));
}

TEST_F(PointCloudRenderableTestFixture, renderable_contains_a_correctly_filled_buffer) {
  size_t vertex_size = renderable_->getBuffer()->getVertexSize();
  size_t number_of_vertices = renderable_->getBuffer()->getNumVertices();

  size_t size_of_single_vertex = 3 * 8 + 4;  // three floats + 4 bytes for color
  size_t vertices_added = 3 * 1;  // three vertices per point

  ASSERT_THAT(vertex_size, Eq(size_of_single_vertex));
  ASSERT_THAT(number_of_vertices, Eq(vertices_added));
  ASSERT_THAT(
    renderable_->getRenderOperation()->operationType, Eq(Ogre::RenderOperation::OT_TRIANGLE_LIST));
}

TEST_F(PointCloudRenderableTestFixture,
  getSquaredViewDepth_returns_squared_length_to_center_of_bounding_box_for_default_camera) {
  Ogre::Camera * camera = Ogre::Root::getSingletonPtr()
    ->createSceneManager()
    ->createCamera("test_camera");
  auto boundingBox = Ogre::AxisAlignedBox(Ogre::Vector3(-1, -1, -1), Ogre::Vector3(3, 3, 1));
  renderable_->setBoundingBox(boundingBox);

  ASSERT_THAT(renderable_->getSquaredViewDepth(camera), Eq(2));
}
