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

#include "rviz_default_plugins/tools/select/selection_tool.hpp"

#include "../tool_test_fixture.hpp"
#include "../../mock_display_context.hpp"
#include "../../mock_selection_manager.hpp"

using namespace ::testing;  // NOLINT

class SelectionToolTestFixture : public ToolTestFixture, public Test
{
public:
  SelectionToolTestFixture()
  {
    context_ = std::make_shared<NiceMock<MockDisplayContext>>();
    selection_manager_ = std::make_shared<NiceMock<MockSelectionManager>>();

    EXPECT_CALL(*context_, getSelectionManager()).WillRepeatedly(Return(selection_manager_));

    render_panel_ = std::make_shared<rviz_common::RenderPanel>(nullptr);
    selection_tool_ = std::make_shared<rviz_default_plugins::tools::SelectionTool>();
    selection_tool_->initialize(context_.get());
  }

  std::shared_ptr<MockDisplayContext> context_;
  std::shared_ptr<MockSelectionManager> selection_manager_;

  std::shared_ptr<rviz_default_plugins::tools::SelectionTool> selection_tool_;
  std::shared_ptr<rviz_common::RenderPanel> render_panel_;
};

TEST_F(SelectionToolTestFixture, processMouseEvent_does_not_render_on_mouse_move) {
  auto event = generateMouseMoveEvent(10, 20);
  auto result = selection_tool_->processMouseEvent(event);

  EXPECT_THAT(result, Eq(0));
}

TEST_F(SelectionToolTestFixture, processMouseEvent_starts_highlighting_on_left_mouse_down) {
  auto event = generateMousePressEvent(10, 20);
  EXPECT_CALL(*selection_manager_, highlight(_, event.x, event.y, event.x, event.y));

  auto result = selection_tool_->processMouseEvent(event);

  EXPECT_THAT(result, Eq(rviz_common::Tool::Render));
}

TEST_F(SelectionToolTestFixture, processMouseEvent_replaces_selection_on_mouse_release) {
  auto click_event = generateMousePressEvent(10, 20);
  selection_tool_->processMouseEvent(click_event);

  auto release_event = generateMouseReleaseEvent(100, 200);
  EXPECT_CALL(
    *selection_manager_, select(
      _,
      click_event.x, click_event.y,
      release_event.x, release_event.y,
      rviz_common::interaction::SelectionManagerIface::Replace));
  selection_tool_->processMouseEvent(release_event);
}

TEST_F(SelectionToolTestFixture, processMouseEvent_adds_to_selection_when_holding_shift) {
  auto click_event = generateMousePressEvent(10, 20);
  selection_tool_->processMouseEvent(click_event);

  auto release_event = generateMouseReleaseEvent(100, 200, Qt::ShiftModifier);
  EXPECT_CALL(
    *selection_manager_, select(
      _,
      click_event.x, click_event.y,
      release_event.x, release_event.y,
      rviz_common::interaction::SelectionManagerIface::Add));
  selection_tool_->processMouseEvent(release_event);
}

TEST_F(SelectionToolTestFixture, processMouseEvent_removes_from_selection_when_holding_ctrl) {
  auto click_event = generateMousePressEvent(10, 20);
  selection_tool_->processMouseEvent(click_event);

  auto release_event = generateMouseReleaseEvent(100, 200, Qt::ControlModifier);
  EXPECT_CALL(
    *selection_manager_, select(
      _,
      click_event.x, click_event.y,
      release_event.x, release_event.y,
      rviz_common::interaction::SelectionManagerIface::Remove));
  selection_tool_->processMouseEvent(release_event);
}

TEST_F(SelectionToolTestFixture, processKeyEvent_F_key_should_focus_on_selection) {
  EXPECT_CALL(*selection_manager_, focusOnSelection()).Times(1);

  QKeyEvent * keyEvent = new QKeyEvent(QKeyEvent::KeyPress, Qt::Key_F, Qt::NoModifier);
  selection_tool_->processKeyEvent(keyEvent, nullptr);
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
