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

#ifndef RVIZ_DEFAULT_PLUGINS__MOCK_VIEW_PICKER_HPP_
#define RVIZ_DEFAULT_PLUGINS__MOCK_VIEW_PICKER_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include <OgreVector.h>

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/render_panel.hpp"


struct Visible3DObject
{
  Visible3DObject(int x, int y, const Ogre::Vector3 & pos)
  : x(x), y(y), position(pos)
  {}

  int x;
  int y;
  Ogre::Vector3 position;
};

class MockViewPicker : public rviz_common::interaction::ViewPickerIface
{
public:
  void initialize() override {}

  bool get3DPoint(rviz_common::RenderPanel * panel, int x, int y, Ogre::Vector3 & pos) override
  {
    (void) panel;
    for (const auto & object : view_objects) {
      if (object.x == x && object.y == y) {
        pos = object.position;
        return true;
      }
    }
    return false;
  }

  bool get3DPatch(
    rviz_common::RenderPanel * panel, int x, int y, unsigned width, unsigned height,
    bool skip_missing, std::vector<Ogre::Vector3> & result_points) override
  {
    (void) panel;
    (void) skip_missing;

    for (const auto & object : view_objects) {
      if (object.x == x && object.y == y) {
        result_points = std::vector<Ogre::Vector3>(width * height, object.position);
        return true;
      }
    }
    return false;
  }

  void registerObject(const Visible3DObject & object)
  {
    view_objects.push_back(object);
  }

private:
  std::vector<Visible3DObject> view_objects;
};

#endif  // RVIZ_DEFAULT_PLUGINS__MOCK_VIEW_PICKER_HPP_
