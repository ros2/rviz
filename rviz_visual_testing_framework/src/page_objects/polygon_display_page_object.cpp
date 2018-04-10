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

#include "rviz_visual_testing_framework/page_objects/polygon_display_page_object.hpp"

#include <memory>
#include <vector>

PolygonDisplayPageObject::PolygonDisplayPageObject(
  int display_id,
  std::shared_ptr<Executor> executor,
  std::shared_ptr<std::vector<int>> all_displays_ids)
: BasePageObject(display_id, 0, "Polygon", executor, all_displays_ids)
{}

void PolygonDisplayPageObject::setTopic(QString topic)
{
  int property_row_index = 1;

  setString("Topic", topic, property_row_index);
}

void PolygonDisplayPageObject::setUnreliable(bool unreliable)
{
  int property_row_index = 2;

  setBool("Unreliable", unreliable, property_row_index);
}

void PolygonDisplayPageObject::setColor(int red, int green, int blue)
{
  int property_row_index = 3;

  QString color_code = QString::fromStdString(
    std::to_string(red) + "; " + std::to_string(green) + "; " + std::to_string(blue));

  setString("Color", color_code, property_row_index);
}

void PolygonDisplayPageObject::setAlpha(QString alpha)
{
  int property_row_index = 4;

  setString("Alpha", alpha, property_row_index);
}
