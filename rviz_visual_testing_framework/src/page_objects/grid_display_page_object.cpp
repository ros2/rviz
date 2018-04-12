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

#include "rviz_visual_testing_framework/page_objects/grid_display_page_object.hpp"

#include <memory>
#include <string>
#include <vector>

#include <QTest>  // NOLINT

GridDisplayPageObject::GridDisplayPageObject(
  int display_id,
  std::shared_ptr<Executor> executor,
  std::shared_ptr<std::vector<int>> all_displays_ids)
: BasePageObject(display_id, 0, "Grid", executor, all_displays_ids)
{}

void GridDisplayPageObject::setReferenceFrame(QString reference_frame)
{
  setComboBox("Reference Frame", reference_frame);
}

void GridDisplayPageObject::setPlaneCellCount(QString plane_cell_count)
{
  setString("Plane Cell Count", plane_cell_count);
}

void GridDisplayPageObject::setNormalCellCount(QString normal_cell_count)
{
  setString("Normal Cell count", normal_cell_count);
}

void GridDisplayPageObject::setCellSize(QString cell_size)
{
  setString("Cell Size", cell_size);
}

void GridDisplayPageObject::setLineStyle(QString line_style)
{
  setComboBox("Line Style", line_style);
}

void GridDisplayPageObject::setColor(int red, int green, int blue)
{
  QString color_code = QString::fromStdString(
    std::to_string(red) + "; " + std::to_string(green) + "; " + std::to_string(blue));

  setString("Color", color_code);
}

void GridDisplayPageObject::setAlpha(QString alpha)
{
  setString("Alpha", alpha);
}

void GridDisplayPageObject::setPlane(QString plane)
{
  setComboBox("Plane", plane);
}

void GridDisplayPageObject::setOffset(float x, float y, float z)
{
  QString offset_triple = format(x) + "; " + format(y) + "; " + format(z);

  setString("Offset", offset_triple);
}
