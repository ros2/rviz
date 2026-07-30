// Copyright (c) 2014, JSK Lab
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gmock/gmock.h>

#include <memory>

#include "sensor_msgs/msg/image.hpp"

#include "rviz_default_plugins/displays/overlay_image/overlay_image_display.hpp"

#include "../display_test_fixture.hpp"

using namespace ::testing;  // NOLINT

namespace
{

sensor_msgs::msg::Image::ConstSharedPtr createImage(uint32_t width, uint32_t height)
{
  auto message = std::make_shared<sensor_msgs::msg::Image>();
  message->width = width;
  message->height = height;
  message->encoding = "rgb8";
  message->step = width * 3;
  message->data.resize(static_cast<size_t>(message->step) * height);
  return message;
}

}  // namespace

class OverlayImageTestFixture : public DisplayTestFixture
{
public:
  OverlayImageTestFixture()
  {
    display_ = std::make_shared<rviz_default_plugins::displays::OverlayImageDisplay>(
      context_.get());
    display_->setName("OverlayImage");
  }

  void setInt(const char * name, int value)
  {
    display_->findProperty(name)->setValue(value);
  }

  std::shared_ptr<rviz_default_plugins::displays::OverlayImageDisplay> display_;
};

TEST_F(OverlayImageTestFixture, dimensions_follow_the_width_and_height_properties) {
  setInt("Width", 320);
  setInt("Height", 240);

  EXPECT_THAT(display_->getOverlayDimensions(createImage(640, 480)), Pair(320, 240));
}

TEST_F(OverlayImageTestFixture, a_negative_dimension_is_taken_from_the_image) {
  setInt("Width", -1);
  setInt("Height", -1);

  EXPECT_THAT(display_->getOverlayDimensions(createImage(640, 480)), Pair(640, 480));
}

TEST_F(OverlayImageTestFixture, a_negative_dimension_without_an_image_falls_back_to_the_default) {
  setInt("Width", -1);
  setInt("Height", -1);

  EXPECT_THAT(display_->getOverlayDimensions(nullptr), Pair(128, 128));
}

TEST_F(OverlayImageTestFixture, keeping_the_aspect_ratio_derives_the_height_from_the_width) {
  display_->findProperty("Keep Aspect Ratio")->setValue(true);
  setInt("Width", 320);
  setInt("Height", 999);

  EXPECT_THAT(display_->getOverlayDimensions(createImage(640, 480)), Pair(320, 240));
  EXPECT_THAT(display_->getOverlayDimensions(createImage(1000, 250)), Pair(320, 80));
}

TEST_F(OverlayImageTestFixture, an_overlay_that_has_never_received_an_image_is_not_pickable) {
  setInt("Left", 40);
  setInt("Top", 40);
  setInt("Width", 320);
  setInt("Height", 240);

  // Before the first message there is nothing on screen, so OverlayPickerTool
  // must not be able to drag the configured rectangle around.
  EXPECT_FALSE(display_->isInRegion(100, 100));
}
