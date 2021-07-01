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
#include <OgreRoot.h>

#include "../ogre_testing_environment.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"

using namespace ::testing;  // NOLINT

class BillboardLineTestFixture : public ::testing::Test
{
protected:
  void SetUp()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;
};

static std::vector<Ogre::Vector3> squareCenteredAtZero
{
  Ogre::Vector3(1, 1, 0),
  Ogre::Vector3(1, -1, 0),
  Ogre::Vector3(-1, 1, 0),
  Ogre::Vector3(-1, -1, 0)
};

// TODO(Martin-Idel-SI): Investigate why returning a stack object here results in errors on Win32
std::unique_ptr<rviz_rendering::BillboardLine> oneCellGrid()
{
  auto grid_cell = std::make_unique<rviz_rendering::BillboardLine>(
    Ogre::Root::getSingletonPtr()->createSceneManager());

  std::vector<Ogre::Vector3> pts = squareCenteredAtZero;
  grid_cell->setMaxPointsPerLine(2);
  grid_cell->setNumLines(2 * 2);  // grid of 1 cell needs 2 vertical and 2 horizontal lines

  grid_cell->addPoint(pts[0]);
  grid_cell->addPoint(pts[1]);
  grid_cell->finishLine();
  grid_cell->addPoint(pts[2]);
  grid_cell->addPoint(pts[3]);
  grid_cell->finishLine();
  grid_cell->addPoint(pts[0]);
  grid_cell->addPoint(pts[2]);
  grid_cell->finishLine();
  grid_cell->addPoint(pts[1]);
  grid_cell->addPoint(pts[3]);

  return grid_cell;
}

std::unique_ptr<rviz_rendering::BillboardLine>
twoLines()
{
  auto two_lines = std::make_unique<rviz_rendering::BillboardLine>(
    Ogre::Root::getSingletonPtr()->createSceneManager());
  std::vector<Ogre::Vector3> pts = squareCenteredAtZero;
  two_lines->setMaxPointsPerLine(2);
  two_lines->setNumLines(2);
  two_lines->addPoint(pts[0]);
  two_lines->addPoint(pts[1]);
  two_lines->finishLine();
  two_lines->addPoint(pts[2]);
  two_lines->addPoint(pts[3]);
  return two_lines;
}

TEST_F(BillboardLineTestFixture, new_billboard_consumes_only_as_much_space_as_necessary) {
  auto grid_cell = oneCellGrid();

  auto chains = grid_cell->getChains();
  ASSERT_THAT(chains, SizeIs(1));
  ASSERT_THAT(chains[0]->getNumberOfChains(), Eq(2u * 2u));
  // A chainElement is exactly a line
  for (unsigned int i = 0; i < chains[0]->getNumberOfChains(); i++) {
    ASSERT_THAT(chains[0]->getNumChainElements(i), Eq(2u));
  }
}

TEST_F(BillboardLineTestFixture, new_billboard_contains_correct_points_as_bounding_box_is_correct) {
  auto grid_cell = oneCellGrid();

  auto chains = grid_cell->getChains();

  // chains are basically bounded by the square
  auto bounding_box = Ogre::AxisAlignedBox(
    Ogre::Vector3(-1.1f, -1.1f, -0.1f), Ogre::Vector3(1.1f, 1.1f, 0.1f));
  ASSERT_THAT(chains[0]->getBoundingBox(), Eq(bounding_box));
}

TEST_F(BillboardLineTestFixture, setColor_results_in_change_of_color_for_all_chains) {
  auto grid_cell = twoLines();

  grid_cell->setColor(0.3f, 0.4f, 0.5f, 0.2f);

  auto chains = grid_cell->getChains();
  for (auto & chain : chains) {
    for (unsigned int i = 0; i < chain->getNumberOfChains(); i++) {
      for (unsigned int j = 0; j < chain->getNumChainElements(i); j++) {
        ASSERT_THAT(
          chain->getChainElement(i, j).colour, Eq(Ogre::ColourValue(0.3f, 0.4f, 0.5f, 0.2f)));
      }
    }
  }
}

TEST_F(BillboardLineTestFixture, setWidth_results_in_change_of_width_for_all_chains) {
  auto grid_cell = twoLines();

  grid_cell->setLineWidth(0.4f);

  auto chains = grid_cell->getChains();
  for (auto & chain : chains) {
    for (unsigned int i = 0; i < chain->getNumberOfChains(); i++) {
      for (unsigned int j = 0; j < chain->getNumChainElements(i); j++) {
        ASSERT_THAT(chain->getChainElement(i, j).width, FloatEq(0.4f));
      }
    }
  }
}

TEST_F(BillboardLineTestFixture, clear_resets_all_chains) {
  auto grid_cell = twoLines();

  grid_cell->clear();

  auto chains = grid_cell->getChains();
  ASSERT_THAT(chains, SizeIs(1));   // chains are reset, not destroyed
  ASSERT_THAT(chains[0]->getNumberOfChains(), Eq(2u));  // reset, not destroyed

  ASSERT_THAT(chains[0]->getNumChainElements(0), Eq(0u));
}

TEST_F(BillboardLineTestFixture, create_multiple_chains) {
  rviz_rendering::BillboardLine grid_cell(Ogre::Root::getSingletonPtr()->createSceneManager());

  grid_cell.setMaxPointsPerLine(2);
  grid_cell.setNumLines(100000);

  for (unsigned int line = 0; line < 100000; line++) {
    if (line != 0) {
      grid_cell.finishLine();
    }
    grid_cell.addPoint(Ogre::Vector3(0, 0, line + 0.1f));
    grid_cell.addPoint(Ogre::Vector3(0, 0, line + 0.3f));
  }

  auto chains = grid_cell.getChains();
  // maximal no of elements per chain: 16384 (defined in billboard_line.cpp)
  ASSERT_THAT(chains, SizeIs(200000 / 16384 + 1));  // +1 to accomodate rest
  ASSERT_THAT(chains[0]->getNumberOfChains(), Eq(16384u / 2u));  // 2 lines per chain
  ASSERT_THAT(chains[0]->getNumChainElements(0), Eq(2u));
}

TEST_F(BillboardLineTestFixture, create_chain_with_extremely_many_elements) {
  rviz_rendering::BillboardLine grid_cell(Ogre::Root::getSingletonPtr()->createSceneManager());

  grid_cell.setMaxPointsPerLine(100000);
  grid_cell.setNumLines(2);

  for (unsigned int line = 0; line < 2; line++) {
    if (line != 0) {
      grid_cell.finishLine();
    }
    for (unsigned int point = 0; point < 100000; point++) {
      grid_cell.addPoint(Ogre::Vector3(0, 0, line + point + 0.1f));
    }
  }

  auto chains = grid_cell.getChains();
  ASSERT_THAT(chains, SizeIs(200000 / 16384 + 1));
  ASSERT_THAT(chains[0]->getNumberOfChains(), Eq(1u));
}
