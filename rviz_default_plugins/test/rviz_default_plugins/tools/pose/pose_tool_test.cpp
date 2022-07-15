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
#include <utility>

#include <OgreVector.h>

#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

#include "../../scene_graph_introspection.hpp"
#include "../tool_test_fixture.hpp"
#include "../../displays/display_test_fixture.hpp"
#include "../../mock_display_context.hpp"

using namespace ::testing;  // NOLINT

class MockProjectionFinder : public rviz_rendering::ViewportProjectionFinder
{
public:
  std::pair<bool, Ogre::Vector3> getViewportPointProjectionOnXYPlane(
    rviz_rendering::RenderWindow * render_window, int x, int y) override
  {
    (void) render_window;
    return {true, Ogre::Vector3(x, y, 0)};
  }
};

class PoseToolImpl : public rviz_default_plugins::tools::PoseTool
{
public:
  PoseToolImpl()
  : PoseTool()
  {
    projection_finder_ = std::make_shared<MockProjectionFinder>();
  }

  ~PoseToolImpl() override = default;

  void onPoseSet(double x, double y, double theta) override
  {
    (void) x;
    (void) y;
    (void) theta;
  }
};

class PoseToolTestFixture : public ToolTestFixture, public DisplayTestFixture
{
public:
  PoseToolTestFixture()
  {
    pose_tool_ = std::make_shared<PoseToolImpl>();
    pose_tool_->initialize(context_.get());
    pose_tool_->activate();
  }

  std::shared_ptr<PoseToolImpl> pose_tool_;
};

TEST_F(PoseToolTestFixture, onInitialize_sets_arrow_invisible_at_first) {
  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());

  ASSERT_THAT(arrows, SizeIs(1));
  EXPECT_FALSE(rviz_default_plugins::arrowIsVisible(arrows[0]));
}

TEST_F(PoseToolTestFixture, processMouseEvent_sets_arrow_position_below_mouse_cursor) {
  auto left_click_event = generateMousePressEvent(10, 15);
  pose_tool_->processMouseEvent(left_click_event);

  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  ASSERT_THAT(arrows, SizeIs(1));
  EXPECT_FALSE(rviz_default_plugins::arrowIsVisible(arrows[0]));
  EXPECT_THAT(arrows[0]->getPosition(), Vector3Eq(Ogre::Vector3(10, 15, 0)));
}

TEST_F(
  PoseToolTestFixture,
  processMouseEvent_sets_arrow_orientation_in_direction_of_the_mouse_while_making_it_visible) {
  auto left_click_event = generateMousePressEvent(10, 10);
  auto move_mouse_event = generateMouseMoveWhileLeftClickedEvent(0, 0);
  pose_tool_->processMouseEvent(left_click_event);
  pose_tool_->processMouseEvent(move_mouse_event);

  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto expected_orientation = Ogre::Quaternion(-0.270598f, -0.653282f, 0.270598f, -0.653282f);
  ASSERT_THAT(arrows, SizeIs(1));
  EXPECT_TRUE(rviz_default_plugins::arrowIsVisible(arrows[0]));
  EXPECT_THAT(arrows[0]->getPosition(), Vector3Eq(Ogre::Vector3(10, 10, 0)));
  EXPECT_THAT(arrows[0]->getOrientation(), QuaternionEq(expected_orientation));
}

TEST_F(PoseToolTestFixture, deactivate_makes_arrow_invisible) {
  auto left_click_event = generateMousePressEvent(10, 10);
  auto move_mouse_event = generateMouseMoveWhileLeftClickedEvent(0, 0);
  pose_tool_->processMouseEvent(left_click_event);
  pose_tool_->processMouseEvent(move_mouse_event);
  pose_tool_->deactivate();

  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  ASSERT_THAT(arrows, SizeIs(1));
  EXPECT_FALSE(rviz_default_plugins::arrowIsVisible(arrows[0]));
}


int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
