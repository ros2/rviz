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

#include <OgreManualObject.h>
#include <OgreSceneNode.h>
#include <OgreMaterialManager.h>
#include <OgreWireBoundingBox.h>

#include <memory>
#include <vector>

#include "rviz_common/selection/selection_handler.hpp"
#include "rviz_common/display_context.hpp"

#include "selection_test_fixture.hpp"

class SelectionHandlerFixture : public SelectionTestFixture
{
public:
  void SetUp() override
  {
    SelectionTestFixture::SetUp();
    handler_ = std::make_shared<rviz_common::selection::SelectionHandler>(context_.get());
  }

  void TearDown() override
  {
    handler_.reset();
    SelectionTestFixture::TearDown();
  }

  Ogre::ManualObject * createManualObject()
  {
    static int count = 0;
    auto object = scene_manager_->createManualObject("ManualObject" + std::to_string(count++));

    object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
    object->position(-1.0f, -1.0f, 0.0f);
    object->position(1.0f, -1.0f, 0.0f);
    object->position(1.0f, 1.0f, 0.0f);
    object->position(-1.0f, 1.0f, 0.0f);
    object->position(-1.0f, -1.0f, 0.0f);
    object->end();

    return object;
  }

  std::shared_ptr<rviz_common::selection::SelectionHandler> handler_;
};

TEST_F(SelectionHandlerFixture, addTrackedObject_works_correctly) {
  auto manual_object = createManualObject();
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(manual_object);

  handler_->addTrackedObjects(scene_manager_->getRootSceneNode());
  handler_->addTrackedObject(manual_object);

  rviz_common::selection::Picked object(handler_->getHandle());
  std::vector<Ogre::AxisAlignedBox> aabbs;
  handler_->getAABBs(object, aabbs);
  EXPECT_THAT(aabbs, SizeIs(2));
}
