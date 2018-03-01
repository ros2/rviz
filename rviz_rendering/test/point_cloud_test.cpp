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
#include <memory>
#include <regex>
#include <vector>

#include <gtest/gtest.h>  // NOLINT

#include <QApplication>  // NOLINT
#include <QMainWindow>  // NOLINT
#include <QTimer>  // NOLINT
#include <QWidget>  // NOLINT

#include "rviz_rendering/point_cloud.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_rendering/custom_parameter_indices.hpp"
#include "test/rviz_rendering/ogre_testing_environment.hpp"

class PointCloudTestFixture : public ::testing::Test
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
PointCloudTestFixture::testing_environment_ = nullptr;

static Ogre::ColourValue colorValue = Ogre::ColourValue(0.5f, 0.5f, 0.5f, 1.0f);

static std::vector<rviz_rendering::PointCloud::Point> squareCenteredAtZero
{
  {Ogre::Vector3(1, 1, 0), colorValue},
  {Ogre::Vector3(1, -1, 0), colorValue},
  {Ogre::Vector3(-1, 1, 0), colorValue},
  {Ogre::Vector3(-1, -1, 0), colorValue}
};

static std::vector<rviz_rendering::PointCloud::Point> singlePointArray
{
  {Ogre::Vector3(1, 2, 3), colorValue}
};

TEST_F(PointCloudTestFixture, addPoints_for_the_first_time_adds_renderable) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();

  pointCloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  auto renderables = pointCloud->getRenderables();
  size_t expected_size = 1;
  ASSERT_EQ(renderables.size(), expected_size);
}

TEST_F(PointCloudTestFixture, addPoints_many_points_gets_a_good_bounding_box_for_points) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();

  pointCloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  ASSERT_EQ(pointCloud->getBoundingBox().getMaximum(), Ogre::Vector3(1, 1, 0));
  ASSERT_EQ(pointCloud->getBoundingBox().getMinimum(), Ogre::Vector3(-1, -1, 0));
}

TEST_F(PointCloudTestFixture, clear_resets_bounding_box_bounding_radius_and_clears_points) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();

  pointCloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  pointCloud->clear();

  auto renderables = pointCloud->getRenderables();
  ASSERT_TRUE(renderables.empty());
  ASSERT_EQ(pointCloud->getBoundingRadius(), 0.0f);
}

TEST_F(PointCloudTestFixture,
  getBoundingRadius_returns_length_to_point_farthest_away_from_origin) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();

  pointCloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  ASSERT_EQ(pointCloud->getBoundingRadius(), Ogre::Math::Sqrt(2));
}

TEST_F(PointCloudTestFixture,
  for_one_point_getBoundingBox_returns_bounding_box_containing_only_one_point) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();

  std::vector<rviz_rendering::PointCloud::Point> points{{Ogre::Vector3(1, -1, 0), colorValue}};
  pointCloud->addPoints(points.begin(), points.end());

  ASSERT_EQ(pointCloud->getBoundingBox().getMaximum(), points[0].position);
  ASSERT_EQ(pointCloud->getBoundingBox().getMinimum(), points[0].position);
}

TEST_F(PointCloudTestFixture, getBoundingBox_adding_points_correctly_expands_bounding_box) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();

  pointCloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());
  pointCloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  ASSERT_EQ(pointCloud->getBoundingBox().getMaximum(), singlePointArray[0].position);
  ASSERT_EQ(pointCloud->getBoundingBox().getMinimum(), squareCenteredAtZero[3].position);
}

TEST_F(PointCloudTestFixture, popPoints_removes_renderable_if_empty) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();
  pointCloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  pointCloud->popPoints(4);

  auto renderables = pointCloud->getRenderables();
  ASSERT_TRUE(renderables.empty());
}

TEST_F(PointCloudTestFixture, popPoints_correctly_limits_bounding_box_on_removing_points) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();
  pointCloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  pointCloud->popPoints(2);

  ASSERT_EQ(pointCloud->getBoundingBox().getMaximum(), squareCenteredAtZero[2].position);
  ASSERT_EQ(pointCloud->getBoundingBox().getMinimum(), squareCenteredAtZero[3].position);
}

TEST_F(PointCloudTestFixture, popPoints_removes_bounding_box_when_completely_empty) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();
  pointCloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  pointCloud->popPoints(4);

  ASSERT_TRUE(pointCloud->getBoundingBox().isNull());
}

TEST_F(PointCloudTestFixture, setHighlightColor_sets_correct_CustomParameter) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();
  pointCloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  pointCloud->setHighlightColor(0.6f, 0.6f, 0.6f);

  auto renderables = pointCloud->getRenderables();
  for (auto const & renderable : renderables) {
    ASSERT_EQ(renderable->getCustomParameter(RVIZ_RENDERING_HIGHLIGHT_PARAMETER),
      Ogre::Vector4(
        static_cast<Ogre::Real>(0.6),
        static_cast<Ogre::Real>(0.6),
        static_cast<Ogre::Real>(0.6),
        static_cast<Ogre::Real>(0)
      )
    );
  }
}

TEST_F(PointCloudTestFixture, setDimensions_changes_dimensions_of_points_and_newly_added_points) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();
  pointCloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  pointCloud->setDimensions(0.1f, 0.2f, 0.3f);

  auto renderables = pointCloud->getRenderables();
  for (auto const & renderable : renderables) {
    ASSERT_EQ(renderable->getCustomParameter(RVIZ_RENDERING_SIZE_PARAMETER),
      Ogre::Vector4(
        static_cast<Ogre::Real>(0.1),
        static_cast<Ogre::Real>(0.2),
        static_cast<Ogre::Real>(0.3),
        static_cast<Ogre::Real>(0)
      )
    );
  }

  pointCloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  renderables = pointCloud->getRenderables();
  for (auto const & renderable : renderables) {
    ASSERT_EQ(renderable->getCustomParameter(RVIZ_RENDERING_SIZE_PARAMETER),
      Ogre::Vector4(
        static_cast<Ogre::Real>(0.1),
        static_cast<Ogre::Real>(0.2),
        static_cast<Ogre::Real>(0.3),
        static_cast<Ogre::Real>(0)
      )
    );
  }
}

TEST_F(PointCloudTestFixture, setRenderMode_changes_material) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();
  pointCloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  pointCloud->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);

  auto renderables = pointCloud->getRenderables();
  for (auto const & renderable : renderables) {
    std::regex regex("^PointCloudMaterial\\d+Point$");
    ASSERT_TRUE(std::regex_match(renderable->getMaterial()->getName(), regex));
  }

  pointCloud->setRenderMode(rviz_rendering::PointCloud::RM_TILES);

  renderables = pointCloud->getRenderables();
  for (auto const & renderable : renderables) {
    std::regex regex("^PointCloudMaterial\\d+Tiles$");
    ASSERT_TRUE(std::regex_match(renderable->getMaterial()->getName(), regex));
  }
}

TEST_F(PointCloudTestFixture,
  setRenderMode_regenerates_renderables_with_different_size_when_geometry_support_changes) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();
  pointCloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  pointCloud->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);

  auto renderables = pointCloud->getRenderables();
  for (auto const & renderable : renderables) {
    size_t number_of_vertices_per_point = 1;
    ASSERT_EQ(renderable->getBuffer()->getNumVertices(), number_of_vertices_per_point);
  }

  pointCloud->setRenderMode(rviz_rendering::PointCloud::RM_BOXES);

  renderables = pointCloud->getRenderables();
  for (auto const & renderable : renderables) {
    size_t number_of_vertices_per_box = 6 * 3 * 2;  // six sides with two triangles each
    ASSERT_EQ(renderable->getBuffer()->getNumVertices(), number_of_vertices_per_box);
  }
}

TEST_F(PointCloudTestFixture, addPoints_adds_new_renderable_whenever_it_is_called) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();
  pointCloud->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);

  pointCloud->addPoints(singlePointArray.begin(), singlePointArray.end());
  pointCloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  auto renderables = pointCloud->getRenderables();
  ASSERT_EQ(renderables.size(), static_cast<size_t>(2));
}


TEST_F(PointCloudTestFixture, addPoints_adds_vertices_with_correct_geometry_when_called) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();
  pointCloud->setRenderMode(rviz_rendering::PointCloud::RM_FLAT_SQUARES);
  size_t number_of_vertices_per_flat_square = 3 * 2;  // two triangles for one square

  pointCloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  auto renderables = pointCloud->getRenderables();
  for (auto const & renderable : renderables) {
    ASSERT_EQ(renderable->getBuffer()->getNumVertices(), number_of_vertices_per_flat_square);
  }

  pointCloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  renderables = pointCloud->getRenderables();
  size_t number_of_vertices_in_all_renderables = 0;
  for (auto const & renderable : renderables) {
    number_of_vertices_in_all_renderables += renderable->getBuffer()->getNumVertices();
  }
  ASSERT_EQ(number_of_vertices_in_all_renderables, number_of_vertices_per_flat_square * 5);
}

TEST_F(PointCloudTestFixture,
  adding_and_removing_points_many_times_does_not_lead_to_superfluous_renderables) {
  auto pointCloud = std::make_shared<rviz_rendering::PointCloud>();
  pointCloud->setRenderMode(rviz_rendering::PointCloud::RM_BOXES);

  for (int i = 0; i < 1000; i++) {
    pointCloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());
  }
  for (int i = 0; i < 99; i++) {
    pointCloud->popPoints(40);
  }

  ASSERT_EQ(pointCloud->getRenderables().size(), static_cast<size_t>(10));
}
