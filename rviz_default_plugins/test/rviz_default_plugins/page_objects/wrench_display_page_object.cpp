/*
 * Copyright (c) 2019, Martin Idel
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

#include "wrench_display_page_object.hpp"

#include <QString>

#include <memory>
#include <vector>

WrenchDisplayPageObject::WrenchDisplayPageObject()
: BasePageObject(0, "Wrench")
{}

void WrenchDisplayPageObject::setTopic(QString topic)
{
  setComboBox("Topic", topic);
  waitForFirstMessage();
}

void WrenchDisplayPageObject::setAlpha(float alpha)
{
  setFloat("Alpha", alpha);
}

void WrenchDisplayPageObject::setForceColor(int r, int g, int b)
{
  setColorCode("Force Color", r, g, b);
}

void WrenchDisplayPageObject::setTorqueColor(int r, int g, int b)
{
  setColorCode("Torque Color", r, g, b);
}

void WrenchDisplayPageObject::setForceScale(float scale)
{
  setFloat("Force Arrow Scale", scale);
}

void WrenchDisplayPageObject::setTorqueScale(float scale)
{
  setFloat("Torque Arrow Scale", scale);
}

void WrenchDisplayPageObject::setWidth(float width)
{
  setFloat("Arrow Width", width);
}

void WrenchDisplayPageObject::setHistoryLength(int history)
{
  setInt("History Length", history);
}
