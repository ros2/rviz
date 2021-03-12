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

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_manager_iface.hpp"

using rviz_common::interaction::CollObjectHandle;
using rviz_common::interaction::M_Picked;
using rviz_common::interaction::SelectionHandler;

class MockSelectionManager : public rviz_common::interaction::SelectionManagerIface
{
public:
  MOCK_METHOD0(initialize, void());
  MOCK_METHOD1(setDebugMode, void(bool));

  MOCK_METHOD5(highlight, void(rviz_rendering::RenderWindow *, int, int, int, int));
  MOCK_METHOD0(removeHighlight, void());
  MOCK_METHOD6(select, void(rviz_rendering::RenderWindow *, int, int, int, int, SelectType));
  MOCK_METHOD6(pick, void(rviz_rendering::RenderWindow *, int, int, int, int, M_Picked &));

  MOCK_METHOD0(update, void());
  MOCK_CONST_METHOD0(getSelection, const M_Picked & ());

  MOCK_METHOD0(focusOnSelection, void());
  MOCK_METHOD1(setTextureSize, void(unsigned int));

  MOCK_METHOD4(get3DPoint, bool(Ogre::Viewport *, int, int, Ogre::Vector3 &));
  MOCK_METHOD7(
    get3DPatch, bool(Ogre::Viewport *, int, int, unsigned, unsigned,
    bool, std::vector<Ogre::Vector3>&));
  MOCK_METHOD0(getPropertyModel, rviz_common::properties::PropertyTreeModel *());
};

#endif  // RVIZ_DEFAULT_PLUGINS__MOCK_SELECTION_MANAGER_HPP_
