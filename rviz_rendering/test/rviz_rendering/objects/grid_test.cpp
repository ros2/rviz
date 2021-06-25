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
#include <string>
#include <vector>

#include <OgreBillboardChain.h>
#include <OgreManualObject.h>
#include <OgreRoot.h>

#include "../ogre_testing_environment.hpp"
#include "rviz_rendering/objects/grid.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"

using namespace ::testing;  // NOLINT

class GridTestFixture : public ::testing::Test
{
protected:
  void SetUp()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;
};

rviz_rendering::Grid createGrid(
  rviz_rendering::Grid::Style style,
  uint32_t cell_count,
  float cell_length,
  float line_width)
{
  Ogre::ColourValue colour_value = Ogre::ColourValue();
  return rviz_rendering::Grid(
    Ogre::Root::getSingletonPtr()->createSceneManager(),
    nullptr, style, cell_count, cell_length, line_width, colour_value);
}

Ogre::AxisAlignedBox expectedBoxForBillboards(
  uint32_t cell_count, uint32_t height_count,
  float cell_length, float line_width)
{
  return Ogre::AxisAlignedBox(
    Ogre::Vector3(
      -cell_length * cell_count / 2 - line_width,
      -cell_length * height_count / 2 - line_width,
      -cell_length * cell_count / 2 - line_width),
    Ogre::Vector3(
      cell_length * cell_count / 2 + line_width,
      cell_length * height_count / 2 + line_width,
      cell_length * cell_count / 2 + line_width));
}

Ogre::AxisAlignedBox expectedBoxForLines(
  uint32_t cell_count, uint32_t height_count,
  float cell_length)
{
  return expectedBoxForBillboards(cell_count, height_count, cell_length, 0);
}

TEST_F(GridTestFixture, createGrid_returns_object_with_bounding_box_bounding_cells) {
  uint32_t cell_count = 2;
  float cell_length = 2.0f;

  auto grid = createGrid(rviz_rendering::Grid::Style::Lines, cell_count, cell_length, 1.0f);

  grid.create();

  auto manual_object = grid.getManualObject();
  auto expected_bounding_box = expectedBoxForLines(cell_count, 0, cell_length);
  ASSERT_THAT(manual_object->getBoundingBox(), Eq(expected_bounding_box));
}

TEST_F(GridTestFixture, setHeight_creates_object_with_correct_height_in_bounding_box) {
  uint32_t cell_count = 3;
  float cell_length = 1.5f;
  uint32_t height_count = 4;

  auto grid = createGrid(rviz_rendering::Grid::Style::Lines, cell_count, cell_length, 1.0f);

  grid.setHeight(height_count);

  auto manual_object = grid.getManualObject();
  auto expected_bounding_box = expectedBoxForLines(cell_count, height_count, cell_length);
  ASSERT_THAT(manual_object->getBoundingBox(), Eq(expected_bounding_box));
}

TEST_F(
  GridTestFixture,
  createGrid_returns_billboard_object_with_bounding_box_bounding_cells_including_billboard_size) {
  uint32_t cell_count = 2;
  float cell_length = 2.0f;
  float line_width = 0.1f;

  auto grid = createGrid(
    rviz_rendering::Grid::Style::Billboards, cell_count, cell_length, line_width);

  grid.create();

  auto billboard_object = grid.getBillboardLine();
  auto expected_bounding_box = expectedBoxForBillboards(cell_count, 0, cell_length, line_width);
  ASSERT_THAT(billboard_object->getChains()[0]->getBoundingBox(), Eq(expected_bounding_box));
}

TEST_F(GridTestFixture, setStyle_creates_a_new_grid_with_new_style) {
  uint32_t cell_count = 2;
  float cell_length = 2.0f;
  float line_width = 0.1f;

  auto grid = createGrid(
    rviz_rendering::Grid::Style::Lines, cell_count, cell_length, line_width);
  grid.create();

  grid.setStyle(rviz_rendering::Grid::Style::Billboards);

  auto billboard_object = grid.getBillboardLine();
  auto expected_bounding_box = expectedBoxForBillboards(cell_count, 0, cell_length, line_width);
  ASSERT_THAT(billboard_object->getChains()[0]->getBoundingBox(), Eq(expected_bounding_box));
  auto manual_object = grid.getManualObject();
  ASSERT_THAT(manual_object->getBoundingBox(), Eq(Ogre::AxisAlignedBox()));
}
