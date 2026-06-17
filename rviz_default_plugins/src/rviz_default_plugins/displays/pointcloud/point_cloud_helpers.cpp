// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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

#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <OgreColourValue.h>

namespace rviz_default_plugins
{

int32_t findChannelIndex(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const std::string & channel)
{
  for (size_t i = 0; i < cloud->fields.size(); ++i) {
    if (cloud->fields[i].name == channel) {
      return static_cast<uint32_t>(i);
    }
  }

  return -1;
}

void getRainbowColor(float value, Ogre::ColourValue & color)
{
  // this is HSV color palette with hue values going only from 0.0 to 0.833333.

  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);

  float h = value * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if (!(i & 1) ) {
    f = 1 - f;             // if i is even
  }
  float n = 1 - f;

  if (i <= 1) {
    color[0] = n, color[1] = 0, color[2] = 1;
  } else if (i == 2) {
    color[0] = 0, color[1] = n, color[2] = 1;
  } else if (i == 3) {
    color[0] = 0, color[1] = 1, color[2] = n;
  } else if (i == 4) {
    color[0] = n, color[1] = 1, color[2] = 0;
  } else if (i >= 5) {
    color[0] = 1, color[1] = n, color[2] = 0;
  }
}

}  // end namespace rviz_default_plugins
