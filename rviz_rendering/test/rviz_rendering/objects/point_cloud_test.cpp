// Copyright (c) 2017, Bosch Software Innovations GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <regex>
#include <string>
#include <vector>

#include <OgreHighLevelGpuProgram.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreRenderSystem.h>
#include <OgreResourceGroupManager.h>
#include <OgreRoot.h>
#include <OgreTechnique.h>

#include <QApplication>  // NOLINT
#include <QMainWindow>  // NOLINT
#include <QTimer>  // NOLINT
#include <QWidget>  // NOLINT

#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_rendering/custom_parameter_indices.hpp"
#include "../ogre_testing_environment.hpp"

#include "../matcher.hpp"

using namespace ::testing;  // NOLINT

class PointCloudTestFixture : public ::testing::Test
{
protected:
  void SetUp()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;
};

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
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  auto renderables = point_cloud->getRenderables();
  ASSERT_THAT(renderables, SizeIs(1u));
}

TEST_F(PointCloudTestFixture, addPoints_many_points_gets_a_good_bounding_box_for_points) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  ASSERT_THAT(
    point_cloud->getBoundingBox(),
    AllOf(
      HasMinimum(Ogre::Vector3(-1, -1, 0)),
      HasMaximum(Ogre::Vector3(1, 1, 0))
  ));
}

TEST_F(PointCloudTestFixture, clear_resets_bounding_box_bounding_radius_and_clears_points) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  point_cloud->clear();

  auto renderables = point_cloud->getRenderables();
  ASSERT_THAT(renderables, IsEmpty());
  ASSERT_THAT(point_cloud->getBoundingRadius(), FloatEq(0.0f));
}

TEST_F(
  PointCloudTestFixture,
  getBoundingRadius_returns_length_to_point_farthest_away_from_origin) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  ASSERT_THAT(point_cloud->getBoundingRadius(), FloatEq(Ogre::Math::Sqrt(2)));
}

TEST_F(
  PointCloudTestFixture,
  for_one_point_getBoundingBox_returns_bounding_box_containing_only_one_point) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  std::vector<rviz_rendering::PointCloud::Point> points{{Ogre::Vector3(1, -1, 0), colorValue}};
  point_cloud->addPoints(points.begin(), points.end());

  ASSERT_THAT(
    point_cloud->getBoundingBox(),
    AllOf(
      HasMinimum(points[0].position),
      HasMaximum(points[0].position)
  ));
}

TEST_F(PointCloudTestFixture, getBoundingBox_adding_points_correctly_expands_bounding_box) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());
  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  ASSERT_THAT(
    point_cloud->getBoundingBox(),
    AllOf(
      HasMinimum(squareCenteredAtZero[3].position),
      HasMaximum(singlePointArray[0].position)
  ));
}

TEST_F(PointCloudTestFixture, popPoints_removes_renderable_if_empty) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  point_cloud->popPoints(4);

  auto renderables = point_cloud->getRenderables();
  ASSERT_THAT(renderables, IsEmpty());
}

TEST_F(PointCloudTestFixture, popPoints_correctly_limits_bounding_box_on_removing_points) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  point_cloud->popPoints(2);
  ASSERT_THAT(
    point_cloud->getBoundingBox(),
    AllOf(
      HasMinimum(squareCenteredAtZero[3].position),
      HasMaximum(squareCenteredAtZero[2].position)
  ));
}

TEST_F(PointCloudTestFixture, popPoints_removes_bounding_box_when_completely_empty) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  point_cloud->popPoints(4);

  ASSERT_TRUE(point_cloud->getBoundingBox().isNull());
}

TEST_F(PointCloudTestFixture, setHighlightColor_sets_correct_CustomParameter) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  point_cloud->setHighlightColor(0.6f, 0.6f, 0.6f);

  auto renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    ASSERT_THAT(
      renderable->getCustomParameter(RVIZ_RENDERING_HIGHLIGHT_PARAMETER),
      Vector4Eq(Ogre::Vector4(0.6f, 0.6f, 0.6f, 0)));
  }
}

TEST_F(PointCloudTestFixture, setDimensions_changes_dimensions_of_points_and_newly_added_points) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  point_cloud->setDimensions(0.1f, 0.2f, 0.3f);

  auto renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    ASSERT_THAT(
      renderable->getCustomParameter(RVIZ_RENDERING_SIZE_PARAMETER),
      Vector4Eq(Ogre::Vector4(0.1f, 0.2f, 0.3f, 0)));
  }

  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    ASSERT_THAT(
      renderable->getCustomParameter(RVIZ_RENDERING_SIZE_PARAMETER),
      Vector4Eq(Ogre::Vector4(0.1f, 0.2f, 0.3f, 0)));
  }
}

TEST_F(PointCloudTestFixture, setRenderMode_changes_material) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);

  auto renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    std::regex regex("^PointCloudMaterial\\d+Point$");
    ASSERT_TRUE(std::regex_match(renderable->getMaterial()->getName(), regex));
  }

  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_TILES);

  renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    std::regex regex("^PointCloudMaterial\\d+Tiles$");
    ASSERT_TRUE(std::regex_match(renderable->getMaterial()->getName(), regex));
  }
}

TEST_F(
  PointCloudTestFixture,
  setRenderMode_regenerates_renderables_with_different_size_when_geometry_support_changes) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);

  auto renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    size_t number_of_vertices_per_point = point_cloud->getVerticesPerPoint();
    ASSERT_THAT(renderable->getBuffer()->getNumVertices(), Eq(number_of_vertices_per_point));
  }

  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_BOXES);

  renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    size_t number_of_vertices_per_box = point_cloud->getVerticesPerPoint();
    ASSERT_THAT(renderable->getBuffer()->getNumVertices(), Eq(number_of_vertices_per_box));
  }
}

TEST_F(PointCloudTestFixture, addPoints_adds_new_renderable_whenever_it_is_called) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);

  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());
  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  auto renderables = point_cloud->getRenderables();
  ASSERT_THAT(renderables, SizeIs(2));
}


TEST_F(PointCloudTestFixture, addPoints_adds_vertices_with_correct_geometry_when_called) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_FLAT_SQUARES);
  size_t number_of_vertices_per_flat_square = point_cloud->getVerticesPerPoint();

  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  auto renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    ASSERT_THAT(renderable->getBuffer()->getNumVertices(), Eq(number_of_vertices_per_flat_square));
  }

  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  renderables = point_cloud->getRenderables();
  size_t number_of_vertices_in_all_renderables = 0;
  for (auto const & renderable : renderables) {
    number_of_vertices_in_all_renderables += renderable->getBuffer()->getNumVertices();
  }
  ASSERT_THAT(number_of_vertices_in_all_renderables, Eq(number_of_vertices_per_flat_square * 5));
}

TEST_F(
  PointCloudTestFixture,
  adding_and_removing_points_many_times_does_not_lead_to_superfluous_renderables) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_BOXES);

  for (int i = 0; i < 1000; i++) {
    point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());
  }
  for (int i = 0; i < 99; i++) {
    point_cloud->popPoints(40);
  }

  ASSERT_THAT(point_cloud->getRenderables(), SizeIs(10));
}

static const std::vector<std::string> kGeometryShaderMaterials = {
  "rviz/PointCloudSquare", "rviz/PointCloudFlatSquare", "rviz/PointCloudSphere",
  "rviz/PointCloudBox"};

static Ogre::MaterialPtr cloneMaterial(const std::string & name)
{
  static int count = 0;
  return Ogre::MaterialManager::getSingleton().getByName(name)->clone(
    name + "Test" + std::to_string(count++));
}

static bool hasGeometryShaderTechnique(const Ogre::MaterialPtr & material)
{
  for (const auto * technique : material->getTechniques()) {
    if (technique->getName() == "gp" || technique->getName().rfind("gp_", 0) == 0) {
      return true;
    }
  }
  return false;
}

TEST_F(PointCloudTestFixture, gp_techniques_draw_the_same_shapes_as_their_nogp_counterparts) {
  for (const auto & material_name : kGeometryShaderMaterials) {
    auto material = Ogre::MaterialManager::getSingleton().getByName(material_name);
    ASSERT_TRUE(material) << material_name;
    for (const std::string suffix :
      {"", "_depth", "_selection_first_pass", "_selection_second_pass"})
    {
      SCOPED_TRACE(material_name + " gp" + suffix);
      auto * gp = material->getTechnique("gp" + suffix);
      auto * nogp = material->getTechnique("nogp" + suffix);
      ASSERT_TRUE(gp && nogp);
      ASSERT_EQ(gp->getSchemeName(), nogp->getSchemeName());
      const auto * gp_pass = gp->getPass(0);
      const auto * nogp_pass = nogp->getPass(0);
      EXPECT_EQ(gp_pass->getFragmentProgramName(), nogp_pass->getFragmentProgramName());
      EXPECT_EQ(gp_pass->getAlphaRejectFunction(), nogp_pass->getAlphaRejectFunction());
      EXPECT_EQ(gp_pass->getAlphaRejectValue(), nogp_pass->getAlphaRejectValue());
    }
  }
}

TEST_F(PointCloudTestFixture, gp_program_parameters_match_the_shader_layout) {
  for (const std::string name : {"rviz/glsl150/billboard.geom",
      "rviz/glsl150/billboard.geom(with_depth)", "rviz/glsl150/box.geom",
      "rviz/glsl150/box.geom(with_depth)"})
  {
    SCOPED_TRACE(name);
    auto program = Ogre::HighLevelGpuProgramManager::getSingleton().getByName(
      name, "rviz_rendering");
    ASSERT_TRUE(program);
    const std::string source = Ogre::ResourceGroupManager::getSingleton().openResource(
      program->getSourceFile(), "rviz_rendering")->getAsString();
    std::smatch layout;
    ASSERT_TRUE(
      std::regex_search(
        source, layout,
        std::regex(R"(layout\((\w+)\) in;\s*layout\((\w+), max_vertices=(\d+)\) out;)")));
    EXPECT_EQ(layout[1].str(), "points");
    EXPECT_EQ(program->getParameter("input_operation_type"), "point_list");
    EXPECT_EQ(layout[2].str(), "triangle_strip");
    EXPECT_EQ(program->getParameter("output_operation_type"), "triangle_strip");
    EXPECT_EQ(program->getParameter("max_output_vertices"), layout[3].str());
  }
}

TEST_F(PointCloudTestFixture, loadPointCloudMaterial_removes_gp_techniques_below_glsl_150) {
  for (const auto & material_name : kGeometryShaderMaterials) {
    SCOPED_TRACE(material_name);
    auto material = cloneMaterial(material_name);

    rviz_rendering::loadPointCloudMaterial(material, 140);

    EXPECT_FALSE(hasGeometryShaderTechnique(material));
    ASSERT_TRUE(material->getBestTechnique());
    EXPECT_EQ(material->getBestTechnique()->getName(), "nogp");
  }
}

TEST_F(
  PointCloudTestFixture,
  loadPointCloudMaterial_removes_all_gp_techniques_if_one_is_not_supported) {
  auto broken = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
    "rviz/test/broken.frag", "rviz_rendering", "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
  broken->setSource("#version 120\nvoid main() { this does not compile }\n");
  auto material = cloneMaterial("rviz/PointCloudSphere");
  material->getTechnique("gp_depth")->getPass(0)->setFragmentProgram("rviz/test/broken.frag");

  rviz_rendering::loadPointCloudMaterial(material, 460);

  EXPECT_FALSE(hasGeometryShaderTechnique(material));
  ASSERT_TRUE(material->getBestTechnique());
  EXPECT_EQ(material->getBestTechnique()->getName(), "nogp");
}

TEST_F(PointCloudTestFixture, loadPointCloudMaterial_keeps_gp_techniques_the_driver_supports) {
  const auto * capabilities = Ogre::Root::getSingleton().getRenderSystem()->getCapabilities();
  if (!capabilities->hasCapability(Ogre::RSC_GEOMETRY_PROGRAM)) {
    GTEST_SKIP() << "The driver does not support geometry shaders";
  }
  for (const auto & material_name : kGeometryShaderMaterials) {
    SCOPED_TRACE(material_name);
    auto material = cloneMaterial(material_name);

    rviz_rendering::loadPointCloudMaterial(material, 150);

    for (const auto * technique : material->getTechniques()) {
      EXPECT_TRUE(technique->isSupported()) << technique->getName();
    }
    ASSERT_TRUE(material->getBestTechnique());
    EXPECT_EQ(material->getBestTechnique()->getName(), "gp");
  }
}
