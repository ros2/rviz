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

#include "depth_cloud_page_object.hpp"

#include <memory>
#include <vector>

DepthCloudDisplayPageObject::DepthCloudDisplayPageObject()
: BasePageObject(0, "DepthCloud")
{}

void DepthCloudDisplayPageObject::setDepthMapTopic(QString topic)
{
  setString("Depth Map Topic", topic);
  waitForFirstMessage();
}

void DepthCloudDisplayPageObject::setImageMapTopic(QString topic)
{
  setString("Color Image Topic", topic);
  waitForFirstMessage();
}

void DepthCloudDisplayPageObject::setQueueSize(int size)
{
  setInt("Queue Size", size);
}

void DepthCloudDisplayPageObject::setDecay(int size)
{
  setInt("Decay Time", size);
}

void DepthCloudDisplayPageObject::setSize(int size)
{
  setInt("Size (m)", size);
}

void DepthCloudDisplayPageObject::setAlpha(float alpha)
{
  setFloat("Alpha", alpha);
}

void DepthCloudDisplayPageObject::setStyle(QString style)
{
  setComboBox("Style", style);
}

void DepthCloudDisplayPageObject::setDepthMapTransport(QString type)
{
  setComboBox("Depth Map Transport Hint", type);
  waitForFirstMessage();
}

void DepthCloudDisplayPageObject::setImageMapTransport(QString type)
{
  setComboBox("Color Transport Hint", type);
  waitForFirstMessage();
}

void DepthCloudDisplayPageObject::setSelectable(bool selectable)
{
  setBool("Selectable", selectable);
}

void DepthCloudDisplayPageObject::setOclusionCompensation(bool visibility)
{
  setBool("Occlusion Compensation", visibility);
}

void DepthCloudDisplayPageObject::setTopicFilter(bool visibility)
{
  setBool("Topic Filter", visibility);
}
