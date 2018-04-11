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

#include "rviz_visual_testing_framework/page_objects/point_cloud_display_page_object.hpp"

#include <memory>
#include <iostream>
#include <vector>

#include <QTest>  // NOLINT

PointCloudDisplayPageObject::PointCloudDisplayPageObject(
  int display_id,
  std::shared_ptr<Executor> executor,
  std::shared_ptr<std::vector<int>> all_displays_ids)
: BasePageObject(display_id, 0, "PointCloud", executor, all_displays_ids)
{}

void PointCloudDisplayPageObject::setSizeMeters(QString meters_size)
{
  int property_row_index = 6;

  setString("Size (m)", meters_size, property_row_index);
}

void PointCloudDisplayPageObject::setSizePixels(QString pixels_size)
{
  int property_row_index = 7;

  setString("Size (Pixels)", pixels_size, property_row_index);
}

void PointCloudDisplayPageObject::setStyle(QString points_style)
{
  int property_row_index = 5;

  setComboBox("Style", points_style, property_row_index);
}

void PointCloudDisplayPageObject::setAlpha(QString alpha)
{
  int property_row_index = 8;

  setString("Alpha", alpha, property_row_index);
}

void PointCloudDisplayPageObject::setDecayTime(QString decay_time)
{
  int property_row_index = 9;

  setString("Decay Time", decay_time, property_row_index);
}

void PointCloudDisplayPageObject::setQueueSize(QString queue_size)
{
  int property_row_index = 3;

  setString("Queue Size", queue_size, property_row_index);
}

void PointCloudDisplayPageObject::setSelectable(bool selectable)
{
  int property_row_index = 4;

  setBool("Selectable", selectable, property_row_index);
}

void PointCloudDisplayPageObject::setUnreliable(bool unreliable)
{
  int property_row_index = 2;

  setBool("Unreliable", unreliable, property_row_index);
}

void PointCloudDisplayPageObject::setPositionTransformer(QString position_transformer)
{
  int property_row_index = 10;

  setComboBox(
    "Position Transformer", position_transformer, property_row_index);
}

void PointCloudDisplayPageObject::setColorTransformer(QString color_transformer)
{
  int property_row_index = 11;
  setComboBox("Color Transformer", color_transformer, property_row_index);
}

void PointCloudDisplayPageObject::setColor(int red, int green, int blue)
{
  setColorTransformer("FlatColor");

  int property_row_index = 15;

  QString color_code = QString::fromStdString(
    std::to_string(red) + "; " + std::to_string(green) + "; " + std::to_string(blue));

  setString("Color", color_code, property_row_index);
}
