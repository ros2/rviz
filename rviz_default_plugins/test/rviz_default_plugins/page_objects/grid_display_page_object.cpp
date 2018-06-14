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

#include "grid_display_page_object.hpp"

#include <memory>
#include <string>
#include <vector>

#include <QTest>  // NOLINT

GridDisplayPageObject::GridDisplayPageObject()
: BasePageObject(0, "Grid")
{}

void GridDisplayPageObject::setReferenceFrame(QString reference_frame)
{
  setComboBox("Reference Frame", reference_frame);
}

void GridDisplayPageObject::setPlaneCellCount(int plane_cell_count)
{
  setInt("Plane Cell Count", plane_cell_count);
}

void GridDisplayPageObject::setNormalCellCount(int normal_cell_count)
{
  setInt("Normal Cell Count", normal_cell_count);
}

void GridDisplayPageObject::setLineWidth(float line_width)
{
  setFloat("Line Width", line_width, {"Line Style"});
}

void GridDisplayPageObject::setCellSize(float cell_size)
{
  setFloat("Cell Size", cell_size);
}

void GridDisplayPageObject::setLineStyle(QString line_style)
{
  setComboBox("Line Style", line_style);
}

void GridDisplayPageObject::setColor(int red, int green, int blue)
{
  setColorCode("Color", red, green, blue);
}

void GridDisplayPageObject::setAlpha(float alpha)
{
  setFloat("Alpha", alpha);
}

void GridDisplayPageObject::setPlane(QString plane)
{
  setComboBox("Plane", plane);
}

void GridDisplayPageObject::setOffset(float x, float y, float z)
{
  setVector("Offset", x, y, z);
}
