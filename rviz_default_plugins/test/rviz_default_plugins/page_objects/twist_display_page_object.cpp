/*
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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
#include "twist_display_page_object.hpp"

#include <QString>

#include <memory>
#include <vector>

TwistDisplayPageObject::TwistDisplayPageObject()
: BasePageObject(0, "TwistStamped")
{}

void TwistDisplayPageObject::setTopic(QString topic)
{
  setComboBox("Topic", topic);
  waitForFirstMessage();
}

void TwistDisplayPageObject::setAlpha(float alpha)
{
  setFloat("Alpha", alpha);
}

void TwistDisplayPageObject::setAngularColor(int r, int g, int b)
{
  setColorCode("Angular Color", r, g, b);
}

void TwistDisplayPageObject::setLinearColor(int r, int g, int b)
{
  setColorCode("Linear Color", r, g, b);
}

void TwistDisplayPageObject::setLinearScale(float scale)
{
  setFloat("Linear Arrow Scale", scale);
}

void TwistDisplayPageObject::setAngularScale(float scale)
{
  setFloat("Angular Arrow Scale", scale);
}

void TwistDisplayPageObject::setWidth(float width)
{
  setFloat("Arrow Width", width);
}

void TwistDisplayPageObject::setHistoryLength(int history)
{
  setInt("History Length", history);
}
