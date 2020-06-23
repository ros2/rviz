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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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
#include <vector>

#include <OgreRoot.h>
#include <OgreEntity.h>
#include <OgreManualObject.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_common/properties/float_property.hpp"

#include "rviz_default_plugins/displays/tf/frame_info.hpp"

#include "../../scene_graph_introspection.hpp"
#include "../display_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class FrameInfoTestFixture : public DisplayTestFixture
{
public:
  FrameInfoTestFixture()
  {
    frame_info_ = std::make_unique<rviz_default_plugins::displays::FrameInfo>(nullptr);
  }

  std::unique_ptr<rviz_default_plugins::displays::FrameInfo> frame_info_;
};

TEST_F(FrameInfoTestFixture, updateArrow_makes_arrow_invisible_if_transform_has_no_translation) {
  auto arrow = std::make_shared<rviz_rendering::Arrow>(scene_manager_);
  frame_info_->parent_arrow_ = arrow.get();
  frame_info_->distance_to_parent_ = 0;
  auto property = std::make_shared<rviz_common::properties::BoolProperty>();
  property->setValue(true);
  frame_info_->enabled_property_ = property.get();

  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  EXPECT_THAT(arrows, SizeIs(1));
  EXPECT_TRUE(rviz_default_plugins::arrowIsVisible(arrows[0]));

  frame_info_->updateParentArrow(Ogre::Vector3::ZERO, Ogre::Vector3::ZERO, 1);

  auto invisible_arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  EXPECT_THAT(invisible_arrows, SizeIs(1));
  EXPECT_FALSE(rviz_default_plugins::arrowIsVisible(invisible_arrows[0]));
}
