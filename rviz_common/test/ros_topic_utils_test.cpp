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


#include <gmock/gmock.h>

#include "rviz_common/ros_topic_utils.hpp"

using rviz_common::isTopicOrServiceHidden;

TEST(IsTopicOrServiceHidden, regular_topics_are_not_hidden) {
  EXPECT_FALSE(isTopicOrServiceHidden("/camera/image"));
  EXPECT_FALSE(isTopicOrServiceHidden("/camera/image/compressed"));
  EXPECT_FALSE(isTopicOrServiceHidden("relative/topic"));
  EXPECT_FALSE(isTopicOrServiceHidden(""));
  EXPECT_FALSE(isTopicOrServiceHidden("/"));
}

TEST(IsTopicOrServiceHidden, underscore_inside_a_token_is_not_hidden) {
  EXPECT_FALSE(isTopicOrServiceHidden("/foo_bar/baz"));
  EXPECT_FALSE(isTopicOrServiceHidden("/foo/bar_baz"));
}

TEST(IsTopicOrServiceHidden, token_starting_with_underscore_is_hidden) {
  EXPECT_TRUE(isTopicOrServiceHidden("/camera/image/_buf_cpu"));
  EXPECT_TRUE(isTopicOrServiceHidden("/_private/topic"));
  EXPECT_TRUE(isTopicOrServiceHidden("/fibonacci/_action/feedback"));
  EXPECT_TRUE(isTopicOrServiceHidden("relative/_hidden"));
  EXPECT_TRUE(isTopicOrServiceHidden("_hidden"));
}
