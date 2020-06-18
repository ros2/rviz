/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
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

#include <OgreManualObject.h>

#include "rviz_common/tool.hpp"

#include "rviz_default_plugins/tools/measure/measure_tool.hpp"

#include "../tool_test_fixture.hpp"
#include "../../displays/display_test_fixture.hpp"
#include "../../scene_graph_introspection.hpp"
#include "../../mock_display_context.hpp"
#include "../../mock_view_picker.hpp"

using namespace ::testing;  // NOLINT

class MeasureToolTestFixture : public ToolTestFixture, public DisplayTestFixture
{
public:
  MeasureToolTestFixture()
  {
    view_picker_ = std::make_shared<MockViewPicker>();

    EXPECT_CALL(*context_, getViewPicker()).WillRepeatedly(Return(view_picker_));

    measure_tool_ = std::make_shared<rviz_default_plugins::tools::MeasureTool>();
    measure_tool_->initialize(context_.get());
  }

  Visible3DObject addVisible3DObject(int x, int y, Ogre::Vector3 pos)
  {
    Visible3DObject object(x, y, pos);
    view_picker_->registerObject(object);
    return object;
  }

  std::shared_ptr<MockViewPicker> view_picker_;

  std::shared_ptr<rviz_default_plugins::tools::MeasureTool> measure_tool_;
};


TEST_F(MeasureToolTestFixture, choosing_two_objects_shows_a_line_and_the_distance) {
  auto obj1 = addVisible3DObject(10, 10, {1.0, 1.0, 0.0});
  auto obj2 = addVisible3DObject(20, 10, {2.0, 1.0, 0.0});
  QString status;

  auto click = generateMouseLeftClick(obj1.x, obj1.y);
  measure_tool_->processMouseEvent(click);
  click = generateMouseLeftClick(obj2.x, obj2.y);
  EXPECT_CALL(*context_, setStatus(_)).WillOnce(SaveArg<0>(&status));
  measure_tool_->processMouseEvent(click);

  auto line = rviz_default_plugins::findOneManualObject(scene_manager_->getRootSceneNode());
  // Use bounding box to indirectly assert the vertices
  ASSERT_TRUE(line->isVisible());
  EXPECT_THAT(line->getBoundingBox().getMinimum(), Vector3Eq(obj1.position));
  EXPECT_THAT(line->getBoundingBox().getMaximum(), Vector3Eq(obj2.position));

  EXPECT_THAT(status.toStdString(), StartsWith("[Length: 1m]"));
}

TEST_F(MeasureToolTestFixture, hovering_over_a_second_object_updates_the_line_and_status) {
  auto obj1 = addVisible3DObject(10, 10, {1.0, 1.0, 0.0});
  auto obj2 = addVisible3DObject(20, 10, {2.0, 1.0, 0.0});
  QString status;

  auto click = generateMouseLeftClick(obj1.x, obj1.y);
  measure_tool_->processMouseEvent(click);
  auto move = generateMouseMoveEvent(obj2.x, obj2.y);
  EXPECT_CALL(*context_, setStatus(_)).WillOnce(SaveArg<0>(&status));
  measure_tool_->processMouseEvent(move);

  auto line = rviz_default_plugins::findOneManualObject(scene_manager_->getRootSceneNode());
  // Use bounding box to indirectly assert the vertices
  ASSERT_TRUE(line->isVisible());
  EXPECT_THAT(line->getBoundingBox().getMinimum(), Vector3Eq(obj1.position));
  EXPECT_THAT(line->getBoundingBox().getMaximum(), Vector3Eq(obj2.position));

  EXPECT_THAT(status.toStdString(), StartsWith("[Length: 1m]"));
}

TEST_F(MeasureToolTestFixture, right_clicking_removes_the_measurement_line) {
  auto obj1 = addVisible3DObject(10, 10, {1.0, 1.0, 0.0});
  auto obj2 = addVisible3DObject(20, 10, {2.0, 1.0, 0.0});

  auto click = generateMouseLeftClick(obj1.x, obj1.y);
  measure_tool_->processMouseEvent(click);
  click = generateMouseLeftClick(obj2.x, obj2.y);
  measure_tool_->processMouseEvent(click);

  auto line = rviz_default_plugins::findOneManualObject(scene_manager_->getRootSceneNode());
  ASSERT_TRUE(line->isVisible());

  click = generateMouseRightClick(5, 5);
  measure_tool_->processMouseEvent(click);

  ASSERT_FALSE(line->isVisible());
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
