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

#ifndef RVIZ_DEFAULT_PLUGINS__MOCK_DISPLAY_CONTEXT_HPP_
#define RVIZ_DEFAULT_PLUGINS__MOCK_DISPLAY_CONTEXT_HPP_

#include <gmock/gmock.h>

#include <memory>

#include "rviz_common/display_context.hpp"
#include "rviz_common/panel_dock_widget.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/window_manager_interface.hpp"
#include "rviz_common/interaction/handler_manager_iface.hpp"
#include "rviz_common/interaction/selection_manager_iface.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

class MockDisplayContext : public rviz_common::DisplayContext
{
public:
  MOCK_CONST_METHOD0(getSceneManager, Ogre::SceneManager * ());
  MOCK_CONST_METHOD0(getWindowManager, rviz_common::WindowManagerInterface * ());
  MOCK_CONST_METHOD0(
    getHandlerManager, std::shared_ptr<rviz_common::interaction::HandlerManagerIface>());
  MOCK_CONST_METHOD0(
    getSelectionManager, std::shared_ptr<rviz_common::interaction::SelectionManagerIface>());
  MOCK_CONST_METHOD0(getViewPicker, std::shared_ptr<rviz_common::interaction::ViewPickerIface>());
  MOCK_CONST_METHOD0(getFrameManager, rviz_common::FrameManagerIface * ());
  MOCK_METHOD0(getTransformationManager, rviz_common::transformation::TransformationManager * ());
  MOCK_CONST_METHOD0(getFixedFrame, QString());
  MOCK_CONST_METHOD0(getFrameCount, uint64_t());
  MOCK_CONST_METHOD0(getDisplayFactory, rviz_common::DisplayFactory * ());
  MOCK_CONST_METHOD0(
    getRosNodeAbstraction, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr());

  MOCK_METHOD2(handleChar, void(QKeyEvent * event, rviz_common::RenderPanel * panel));
  MOCK_METHOD1(handleMouseEvent, void(const rviz_common::ViewportMouseEvent & even));

  MOCK_CONST_METHOD0(getToolManager, rviz_common::ToolManager * ());
  MOCK_CONST_METHOD0(getViewManager, rviz_common::ViewManager * ());
  MOCK_CONST_METHOD0(getRootDisplayGroup, rviz_common::DisplayGroup * ());

  MOCK_CONST_METHOD0(getDefaultVisibilityBit, uint32_t());
  MOCK_METHOD0(visibilityBits, rviz_common::BitAllocator * ());

  MOCK_METHOD1(setStatus, void(const QString & message));
  MOCK_METHOD0(getClock, std::shared_ptr<rclcpp::Clock>());

  MOCK_METHOD0(queueRender, void());

  MOCK_METHOD0(lockRender, void());
  MOCK_METHOD0(unlockRender, void());
  MOCK_CONST_METHOD0(getHelpPath, QString());
};

#endif  // RVIZ_DEFAULT_PLUGINS__MOCK_DISPLAY_CONTEXT_HPP_
