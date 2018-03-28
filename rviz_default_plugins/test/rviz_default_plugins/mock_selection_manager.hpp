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

#ifndef RVIZ_DEFAULT_PLUGINS__MOCK_SELECTION_MANAGER_HPP_
#define RVIZ_DEFAULT_PLUGINS__MOCK_SELECTION_MANAGER_HPP_

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "rviz_common/selection/forwards.hpp"
#include "rviz_common/selection/selection_manager_iface.hpp"

using rviz_common::selection::CollObjectHandle;
using rviz_common::selection::M_Picked;
using rviz_common::selection::SelectionHandler;

class MockSelectionManager : public rviz_common::selection::SelectionManagerIface
{
public:
  MOCK_METHOD0(createHandle, rviz_common::selection::CollObjectHandle());
  MOCK_METHOD3(renderQueueStarted, void(uint8_t, const std::string &, bool &));

  MOCK_METHOD0(initialize, void());
  MOCK_METHOD1(setDebugMode, void(bool));
  MOCK_METHOD0(clearHandlers, void());

  MOCK_METHOD2(addObject, void(CollObjectHandle, SelectionHandler *));
  MOCK_METHOD1(removeObject, void(rviz_common::selection::CollObjectHandle));
  MOCK_METHOD5(highlight, void(rviz_rendering::RenderWindow *, int, int, int, int));
  MOCK_METHOD0(removeHighlight, void());
  MOCK_METHOD6(select, void(rviz_rendering::RenderWindow *, int, int, int, int, SelectType));
  MOCK_METHOD7(pick, void(rviz_rendering::RenderWindow *, int, int, int, int, M_Picked &, bool));

  MOCK_METHOD0(update, void());
  MOCK_METHOD1(setSelection, void(const M_Picked &));
  MOCK_METHOD1(addSelection, void(const M_Picked &));
  MOCK_METHOD1(removeSelection, void(const M_Picked &));
  MOCK_CONST_METHOD0(getSelection, const M_Picked &());

  MOCK_METHOD1(getHandler, SelectionHandler * (CollObjectHandle));
  MOCK_METHOD5(handleSchemeNotFound, Ogre::Technique *(
      unsigned short int, // NOLINT: Ogre decides the use of unsigned short
      const Ogre::String &,
      Ogre::Material *,
      unsigned short int, // NOLINT: Ogre decides the use of unsigned short
      const Ogre::Renderable *));

  MOCK_METHOD1(enableInteraction, void(bool));
  MOCK_CONST_METHOD0(getInteractionEnabled, bool());
  MOCK_METHOD0(focusOnSelection, void());
  MOCK_METHOD1(setTextureSize, void(unsigned int));

  MOCK_METHOD4(get3DPoint, bool(Ogre::Viewport *, int, int, Ogre::Vector3 &));
  MOCK_METHOD7(get3DPatch, bool(Ogre::Viewport *, int, int, unsigned int, unsigned int, bool,
    std::vector<Ogre::Vector3>&));
  MOCK_METHOD6(getPatchDepthImage, bool(Ogre::Viewport *, int, int, unsigned int, unsigned int,
    std::vector<float, std::allocator<float>>&));
  MOCK_METHOD0(getPropertyModel, rviz_common::properties::PropertyTreeModel *());
};

#endif  // RVIZ_DEFAULT_PLUGINS__MOCK_SELECTION_MANAGER_HPP_
