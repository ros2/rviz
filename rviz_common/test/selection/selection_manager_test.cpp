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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "rviz_common/selection/selection_manager.hpp"
#include "rviz_common/selection/selection_handler.hpp"
#include "rviz_common/display_context.hpp"

#include "../display_context_fixture.hpp"
#include "mock_selection_renderer.hpp"

using namespace ::testing;  // NOLINT

class SelectionManagerTestFixture : public DisplayContextFixture
{
public:
  void SetUp() override
  {
    DisplayContextFixture::SetUp();
    renderer_ = std::make_shared<MockSelectionRenderer>();
    selection_manager_ = std::make_unique<rviz_common::selection::SelectionManager>(
      context_.get(), renderer_);
    selection_manager_->initialize();
    EXPECT_CALL(*context_, getSelectionManager()).WillRepeatedly(
      testing::Return(selection_manager_.get()));
  }

  void TearDown() override
  {
    renderer_.reset();  // necessary for correct order of deleting node
    selection_manager_.reset();  // necessary for correct order of deleting node
    DisplayContextFixture::TearDown();
  }

  VisibleObject addVisibleObject(int x, int y)
  {
    VisibleObject object(x, y, context_.get());
    renderer_->addVisibleObject(object);
    return object;
  }

  std::shared_ptr<MockSelectionRenderer> renderer_;
  std::unique_ptr<rviz_common::selection::SelectionManager> selection_manager_;
};

TEST_F(SelectionManagerTestFixture, pick_picks_objects_from_scene) {
  auto o1 = addVisibleObject(10, 10);
  auto o2 = addVisibleObject(40, 50);

  selection_manager_->select(
    nullptr, 0, 0, 100, 100, rviz_common::selection::SelectionManager::Replace);

  auto picked = selection_manager_->getSelection();
  EXPECT_THAT(picked.size(), Eq(2u));
  EXPECT_THAT(picked, Contains(Key(o1.getHandle())));
  EXPECT_THAT(picked, Contains(Key(o2.getHandle())));
}

TEST_F(SelectionManagerTestFixture, pick_does_not_pick_objects_outside_selection) {
  auto o1 = addVisibleObject(10, 10);
  auto o2 = addVisibleObject(150, 150);

  selection_manager_->select(
    nullptr, 0, 0, 100, 100, rviz_common::selection::SelectionManager::Replace);

  auto picked = selection_manager_->getSelection();
  EXPECT_THAT(picked.size(), Eq(1u));
  EXPECT_THAT(picked, Contains(Key(o1.getHandle())));
  EXPECT_THAT(picked, Not(Contains(Key(o2.getHandle()))));
}
