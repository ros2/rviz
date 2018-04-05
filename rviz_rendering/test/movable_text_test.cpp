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

#include <memory>
#include <string>
#include <vector>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#else
# pragma warning(push)
# pragma warning(disable : 4251)
#endif
#include <OgreMovableObject.h>
#include <OgreFont.h>
#include <OgreFontManager.h>
#include <OgreVector3.h>
#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

#include "test/rviz_rendering/ogre_testing_environment.hpp"
#include "rviz_rendering/objects/movable_text.hpp"

class MovableTextTestFixture : public ::testing::Test
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
MovableTextTestFixture::testing_environment_ = nullptr;

float getCharWidth(std::shared_ptr<rviz_rendering::MovableText> movable_text, char character)
{
  auto font =
    Ogre::FontManager::getSingleton().getByName("Liberation Sans", "rviz_rendering").get();
  font->load();
  float char_height = movable_text->getCharacterHeight();
  float additional_space = 2.0f;
  return font->getGlyphAspectRatio(character) * char_height * additional_space;
}

void assertVector3Equality(Ogre::Vector3 actual, Ogre::Vector3 expected)
{
  ASSERT_NEAR(actual.x, expected.x, 0.0001f);
  ASSERT_NEAR(actual.y, expected.y, 0.0001f);
  ASSERT_NEAR(actual.z, expected.z, 0.0001f);
}
void assertBoundingBoxEquality(Ogre::AxisAlignedBox actual, Ogre::AxisAlignedBox expected)
{
  assertVector3Equality(actual.getMaximum(), expected.getMaximum());
  assertVector3Equality(actual.getMinimum(), expected.getMinimum());
}

TEST_F(MovableTextTestFixture, setCaption_changes_displayed_text) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("TestCaption");

  movable_text->setCaption("NewCaption");

  ASSERT_EQ(movable_text->getCaption(), "NewCaption");
}

TEST_F(MovableTextTestFixture, bounding_box_for_one_character_text_depends_on_glyp_ratio) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("W");

  float char_width = getCharWidth(movable_text, 'W');
  assertBoundingBoxEquality(movable_text->getBoundingBox(),
    Ogre::AxisAlignedBox(Ogre::Vector3(0, -2, 0), Ogre::Vector3(char_width, 0, 0)));
}

TEST_F(MovableTextTestFixture, bounding_box_space_is_given_by_bounding_box_for_A) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("A A");

  float char_width = getCharWidth(movable_text, 'A');
  assertBoundingBoxEquality(movable_text->getBoundingBox(),
    Ogre::AxisAlignedBox(Ogre::Vector3(0, -2, 0), Ogre::Vector3(3 * char_width, 0, 0)));
}

TEST_F(MovableTextTestFixture, new_line_creates_a_new_line_making_bounding_box_larger) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("A\nA");

  float char_width = getCharWidth(movable_text, 'A');
  assertBoundingBoxEquality(movable_text->getBoundingBox(),
    Ogre::AxisAlignedBox(Ogre::Vector3(0, -4 - 0.01f, 0), Ogre::Vector3(char_width, 0, 0)));
}

TEST_F(MovableTextTestFixture, larger_char_height_makes_characters_wider) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("A", "Liberation Sans", 2.0f);

  float char_width = getCharWidth(movable_text, 'A');
  assertBoundingBoxEquality(movable_text->getBoundingBox(),
    Ogre::AxisAlignedBox(Ogre::Vector3(0, -2 * 2.0f, 0), Ogre::Vector3(char_width, 0, 0)));
}

TEST_F(MovableTextTestFixture, horizontal_alignment_center_centers_x_coordinate) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("W");

  movable_text->setTextAlignment(rviz_rendering::MovableText::HorizontalAlignment::H_CENTER,
    rviz_rendering::MovableText::VerticalAlignment::V_BELOW);
  movable_text->update();

  float char_width = getCharWidth(movable_text, 'W');
  assertBoundingBoxEquality(movable_text->getBoundingBox(),
    Ogre::AxisAlignedBox(Ogre::Vector3(-char_width / 2, -2, 0),
    Ogre::Vector3(char_width / 2, 0, 0)));
}

TEST_F(MovableTextTestFixture, vertical_alignment_center_centers_y_coordinate) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("W");

  movable_text->setTextAlignment(rviz_rendering::MovableText::HorizontalAlignment::H_LEFT,
    rviz_rendering::MovableText::VerticalAlignment::V_CENTER);
  movable_text->update();

  float char_width = getCharWidth(movable_text, 'W');
  assertBoundingBoxEquality(movable_text->getBoundingBox(),
    Ogre::AxisAlignedBox(Ogre::Vector3(0, -1, 0), Ogre::Vector3(char_width, 1, 0)));
}

TEST_F(MovableTextTestFixture, vertical_alignment_above_puts_y_coordinate_above_zero) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("W");

  movable_text->setTextAlignment(rviz_rendering::MovableText::HorizontalAlignment::H_LEFT,
    rviz_rendering::MovableText::VerticalAlignment::V_ABOVE);
  movable_text->update();

  float char_width = getCharWidth(movable_text, 'W');
  assertBoundingBoxEquality(movable_text->getBoundingBox(),
    Ogre::AxisAlignedBox(Ogre::Vector3(0, 0, 0), Ogre::Vector3(char_width, 2, 0)));
}

TEST_F(MovableTextTestFixture, vertical_alignment_above_puts_y_coordinate_above) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("W\nW");

  movable_text->setTextAlignment(rviz_rendering::MovableText::HorizontalAlignment::H_LEFT,
    rviz_rendering::MovableText::VerticalAlignment::V_ABOVE);
  movable_text->update();

  float char_width = getCharWidth(movable_text, 'W');
  assertBoundingBoxEquality(movable_text->getBoundingBox(),
    Ogre::AxisAlignedBox(Ogre::Vector3(0, 0, 0), Ogre::Vector3(char_width, 4.01f, 0)));
}

TEST_F(MovableTextTestFixture, setSpaceWidth_changes_width_of_space_character) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("A A");

  float new_space_width = 0.5f;
  movable_text->setSpaceWidth(new_space_width);
  movable_text->update();

  float char_width = getCharWidth(movable_text, 'A');
  assertBoundingBoxEquality(movable_text->getBoundingBox(),
    Ogre::AxisAlignedBox(Ogre::Vector3(0, -2, 0),
    Ogre::Vector3(2 * char_width + new_space_width, 0, 0)));
}

TEST_F(MovableTextTestFixture, setLineSpacing_changes_space_between_lines) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("A\nA");

  float new_line_spacing = 0.5f;
  movable_text->setLineSpacing(new_line_spacing);
  movable_text->update();

  float char_width = getCharWidth(movable_text, 'A');
  assertBoundingBoxEquality(movable_text->getBoundingBox(),
    Ogre::AxisAlignedBox(Ogre::Vector3(0, -2 * 2 - new_line_spacing, 0),
    Ogre::Vector3(char_width, 0, 0)));
}

TEST_F(MovableTextTestFixture, horizontal_alignment_works_correctly_with_spaces) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("A A");

  movable_text->setTextAlignment(rviz_rendering::MovableText::HorizontalAlignment::H_CENTER,
    rviz_rendering::MovableText::VerticalAlignment::V_BELOW);
  movable_text->update();

  float char_width = getCharWidth(movable_text, 'A');
  assertBoundingBoxEquality(movable_text->getBoundingBox(),
    Ogre::AxisAlignedBox(Ogre::Vector3(-3 * char_width / 2, -2, 0),
    Ogre::Vector3(3 * char_width / 2, 0, 0)));
}

TEST_F(MovableTextTestFixture, getBoundingRadius_gets_squared_length_from_origin_to_box_corner) {
  auto movable_text = std::make_shared<rviz_rendering::MovableText>("A A");

  float char_width = getCharWidth(movable_text, 'A');
  Ogre::Vector3 farthest_point = Ogre::Vector3(3 * char_width, -2.0f, 0);
  ASSERT_EQ(movable_text->getBoundingRadius(), Ogre::Math::Sqrt(farthest_point.squaredLength()));
}
