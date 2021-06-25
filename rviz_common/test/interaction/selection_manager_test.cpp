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
#include <string>
#include <vector>

#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/interaction/selection_handler.hpp"
#include "rviz_common/display_context.hpp"

#include "selection_test_fixture.hpp"
#include "mock_selection_renderer.hpp"

using namespace ::testing;  // NOLINT

class SelectionManagerTestFixture : public SelectionTestFixture
{
public:
  VisibleObject addVisibleObject(int x, int y)
  {
    VisibleObject object(x, y, context_.get());
    renderer_->addVisibleObject(object);
    return object;
  }

  ~SelectionManagerTestFixture()
  {
    if (render_window_) {
      delete render_window_;
    }
  }

  rviz_rendering::RenderWindow * render_window_ = nullptr;
};

TEST_F(SelectionManagerTestFixture, select_selects_objects_inside_selection) {
  auto o1 = addVisibleObject(10, 10);
  auto o2 = addVisibleObject(150, 150);

  selection_manager_->select(
    render_window_, 0, 0, 100, 100, rviz_common::interaction::SelectionManager::Replace);

  auto selection = selection_manager_->getSelection();
  EXPECT_THAT(selection, SizeIs(1));
  EXPECT_THAT(selection, Contains(Key(o1.getHandle())));
  EXPECT_THAT(selection, Not(Contains(Key(o2.getHandle()))));
}

TEST_F(SelectionManagerTestFixture, adding_a_new_selection) {
  auto o1 = addVisibleObject(10, 10);
  auto o2 = addVisibleObject(20, 20);
  selection_manager_->select(
    render_window_, 0, 0, 15, 15, rviz_common::interaction::SelectionManager::Replace);

  selection_manager_->select(
    render_window_, 15, 15, 25, 25, rviz_common::interaction::SelectionManager::Add);

  auto selection = selection_manager_->getSelection();
  EXPECT_THAT(selection, SizeIs(2));
  EXPECT_THAT(selection, Contains(Key(o1.getHandle())));
  EXPECT_THAT(selection, Contains(Key(o2.getHandle())));
}

TEST_F(SelectionManagerTestFixture, adding_an_exising_selection_has_no_effect) {
  auto o1 = addVisibleObject(10, 10);
  selection_manager_->select(
    render_window_, 0, 0, 15, 15, rviz_common::interaction::SelectionManager::Replace);

  selection_manager_->select(
    render_window_, 0, 0, 15, 15, rviz_common::interaction::SelectionManager::Add);

  auto selection = selection_manager_->getSelection();
  EXPECT_THAT(selection, SizeIs(1));
  EXPECT_THAT(selection, Contains(Key(o1.getHandle())));
}

TEST_F(SelectionManagerTestFixture, subtracting_from_a_selection) {
  auto o1 = addVisibleObject(10, 10);
  auto o2 = addVisibleObject(20, 20);
  selection_manager_->select(
    render_window_, 0, 0, 25, 25, rviz_common::interaction::SelectionManager::Replace);

  selection_manager_->select(
    render_window_, 0, 0, 15, 15, rviz_common::interaction::SelectionManager::Remove);

  auto selection = selection_manager_->getSelection();
  EXPECT_THAT(selection, SizeIs(1));
  EXPECT_THAT(selection, Contains(Key(o2.getHandle())));
}
