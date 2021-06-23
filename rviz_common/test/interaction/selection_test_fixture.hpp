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

#ifndef INTERACTION__SELECTION_TEST_FIXTURE_HPP_
#define INTERACTION__SELECTION_TEST_FIXTURE_HPP_

#include <gmock/gmock.h>

#include <memory>

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/interaction/handler_manager.hpp"

#include "../display_context_fixture.hpp"
#include "mock_selection_renderer.hpp"

using namespace ::testing;  // NOLINT

class SelectionTestFixture : public DisplayContextFixture
{
public:
  void SetUp()
  {
    DisplayContextFixture::SetUp();
    selection_manager_->initialize();
  }

  SelectionTestFixture()
  {
    renderer_ = std::make_shared<MockSelectionRenderer>(context_.get());
    handler_manager_ = std::make_shared<rviz_common::interaction::HandlerManager>();
    std::weak_ptr<rviz_common::interaction::HandlerManagerIface>
    handler_manager_weak_ptr(handler_manager_);
    selection_manager_ = std::make_shared<rviz_common::interaction::SelectionManager>(
      context_.get(), renderer_);
    std::weak_ptr<rviz_common::interaction::SelectionManagerIface>
    selection_manager_weak_ptr(selection_manager_);
    EXPECT_CALL(*context_, getHandlerManager()).WillRepeatedly(
      Invoke([handler_manager_weak_ptr]() {return handler_manager_weak_ptr.lock();}));
    EXPECT_CALL(*context_, getSelectionManager()).WillRepeatedly(
      Invoke([selection_manager_weak_ptr]() {return selection_manager_weak_ptr.lock();}));
  }

  std::shared_ptr<MockSelectionRenderer> renderer_;
  std::shared_ptr<rviz_common::interaction::HandlerManagerIface> handler_manager_;
  std::shared_ptr<rviz_common::interaction::SelectionManagerIface> selection_manager_;
};

#endif  // INTERACTION__SELECTION_TEST_FIXTURE_HPP_
