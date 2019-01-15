/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__ARROW_MARKER_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__ARROW_MARKER_HPP_

#include <memory>

#include "marker_base.hpp"

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

// This is necessary because of using stl types with this display. Nevertheless, if you are
// experiencing problems when subclassing this class, please make sure ROS2 and your code were
// compiled with the same compiler and version
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace Ogre
{
class SceneNode;
}  // namespace Ogre
namespace rviz_common
{
class DisplayContext;
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC ArrowMarker : public MarkerBase
{
public:
  ArrowMarker(
    MarkerCommon * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node);

  ~ArrowMarker() override = default;

  S_MaterialPtr getMaterials() override;

protected:
  void onNewMessage(
    const MarkerConstSharedPtr & old_message, const MarkerConstSharedPtr & new_message) override;
  std::unique_ptr<rviz_rendering::Arrow> arrow_;
  bool last_arrow_set_from_points_;

private:
  void printErrorMessage();
  virtual void setDefaultProportions();
  void setArrowFromPoints(const MarkerConstSharedPtr & message);
  void setArrow(const MarkerConstSharedPtr & message);
};

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__ARROW_MARKER_HPP_
