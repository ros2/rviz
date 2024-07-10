// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.
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


#ifndef RVIZ_DEFAULT_PLUGINS__PAGE_OBJECTS__PATH_DISPLAY_PAGE_OBJECT_HPP_
#define RVIZ_DEFAULT_PLUGINS__PAGE_OBJECTS__PATH_DISPLAY_PAGE_OBJECT_HPP_

#include "rviz_visual_testing_framework/page_objects/base_page_object.hpp"

class PathDisplayPageObject : public BasePageObject
{
public:
  PathDisplayPageObject();

  void setTopic(QString topic);

  void setLineStyleForPath(QString line_style);
  void setLineWidthForPath(float width);
  void setPathColor(int red, int green, int blue);
  void setAlpha(float alpha);
  void setLengthOfBuffer(int buffer_length);
  void setOffsetFromOrigin(float x, float y, float z);

  void setStyleOfPose(QString pose_style);
  void setPoseAxesLength(float length);
  void setPoseAxesRadius(float radius);
  void setPoseArrowColorProperty(int red, int green, int blue);
  void setPoseArrowShaftLength(float length);
  void setPoseArrowHeadLength(float length);
  void setPoseArrowShaftDiameter(float diameter);
  void setPoseArrowHeadDiameter(float diameter);
};

#endif  // RVIZ_DEFAULT_PLUGINS__PAGE_OBJECTS__PATH_DISPLAY_PAGE_OBJECT_HPP_
