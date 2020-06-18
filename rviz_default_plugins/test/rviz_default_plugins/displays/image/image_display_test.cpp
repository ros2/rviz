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

#include <gmock/gmock.h>

#include <memory>
#include <utility>

#include <QApplication>  // NOLINT
#include <QKeyEvent>  // NOLINT
#include <OgreRectangle2D.h>  // NOLINT

#include "../../ogre_testing_environment.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/panel_dock_widget.hpp"
#include "rviz_common/window_manager_interface.hpp"

#include "./mock_ros_image_texture.hpp"
#include "../../mock_display_context.hpp"
#include "../../mock_window_manager_interface.hpp"

#include "rviz_default_plugins/displays/image/image_display.hpp"

using namespace ::testing;  // NOLINT
using namespace rviz_default_plugins::displays;  // NOLINT

class ImageDisplayTestFixture : public Test
{
public:
  static void SetUpTestCase()
  {
    testing_environment_ = std::make_shared<rviz_default_plugins::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  ImageDisplayTestFixture()
  {
    texture_ = std::make_unique<MockROSImageTexture>();
    ON_CALL(*texture_, getName()).WillByDefault(Return("texture"));

    context_ = std::make_shared<NiceMock<MockDisplayContext>>();

    window_manager_ = std::make_shared<MockWindowManagerInterface>();
    ON_CALL(*context_, getWindowManager()).WillByDefault(Return(window_manager_.get()));
  }

  static std::shared_ptr<rviz_default_plugins::OgreTestingEnvironment> testing_environment_;

  std::unique_ptr<MockROSImageTexture> texture_;

  std::shared_ptr<MockDisplayContext> context_;
  std::shared_ptr<MockWindowManagerInterface> window_manager_;
};

std::shared_ptr<rviz_default_plugins::OgreTestingEnvironment>
ImageDisplayTestFixture::testing_environment_ = nullptr;

TEST_F(ImageDisplayTestFixture, initialize_adds_render_panel_to_window) {
  auto panelDockWidget = new rviz_common::PanelDockWidget("panelDockWidget");
  EXPECT_CALL(*window_manager_, addPane(_, _, _, _)).WillOnce(Return(panelDockWidget));
  EXPECT_CALL(*context_, getFixedFrame()).WillOnce(Return(""));

  ImageDisplay imageDisplay(std::move(texture_));
  imageDisplay.initialize(context_.get());
}

TEST_F(ImageDisplayTestFixture, update_calls_texture_update) {
  auto panelDockWidget = new rviz_common::PanelDockWidget("panelDockWidget");
  EXPECT_CALL(*window_manager_, addPane(_, _, _, _)).WillOnce(Return(panelDockWidget));
  EXPECT_CALL(*context_, getFixedFrame()).WillOnce(Return(""));

  EXPECT_CALL(*texture_, update()).Times(1);

  ImageDisplay imageDisplay(std::move(texture_));
  imageDisplay.initialize(context_.get());
  imageDisplay.update(0, 0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
