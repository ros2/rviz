/*
 * Copyright (c) 2020, TNG Technology Consulting GmbH.
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

#include <gmock/gmock.h>
#include <string>

#include "rviz_default_plugins/displays/image/get_transport_from_topic.hpp"

using namespace rviz_default_plugins::displays;  //  NOLINT

TEST(getTransportFromTopicTest, get_transport_from_topic_finds_right_transport)
{
  EXPECT_EQ(getTransportFromTopic("/image/compressed"), "compressed");
  EXPECT_EQ(getTransportFromTopic("/image_transport"), "raw");
  EXPECT_EQ(getTransportFromTopic("/image_compressed"), "raw");
  EXPECT_EQ(getTransportFromTopic("/image_transport/camera/compressedDepth"), "compressedDepth");
  EXPECT_EQ(getTransportFromTopic("/topic_name/publisher_name/compressed_and_theora"), "raw");
}

TEST(getBaseTopicFromTopicTest, get_transport_from_topic_finds_right_topic)
{
  EXPECT_EQ(getBaseTopicFromTopic("/image/compressed"), "/image");
  EXPECT_EQ(getBaseTopicFromTopic("/image_transport"), "/image_transport");
  EXPECT_EQ(getBaseTopicFromTopic("/image_compressed"), "/image_compressed");
  EXPECT_EQ(
    getBaseTopicFromTopic(
      "/image_transport/camera/compressedDepth"), "/image_transport/camera");
  EXPECT_EQ(
    getBaseTopicFromTopic(
      "/topic_name/publisher_name/compressed_and_theora"),
    "/topic_name/publisher_name/compressed_and_theora");
}
