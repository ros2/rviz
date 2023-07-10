/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <QString>

#include \
  "rviz_default_plugins/displays/interactive_markers/interactive_marker_namespace_property.hpp"

TEST(InteractiveMarkerNamespacePropertyConstructorTest, constructor)
{
  // With defaults
  {
    rviz_default_plugins::displays::InteractiveMarkerNamespaceProperty prop("Test Name");
  }

  // No defaults
  {
    rviz_default_plugins::displays::InteractiveMarkerNamespaceProperty prop(
      "Test Name",
      "Test default value",
      "Test description",
      nullptr,
      nullptr);
  }
}

TEST(InteractiveMarkerNamespacePropertyTest, get_namespace)
{
  QString default_value("Test default value");
  rviz_default_plugins::displays::InteractiveMarkerNamespaceProperty prop(
    "Test Name", default_value);
  EXPECT_EQ(prop.getNamespace(), default_value);
  EXPECT_EQ(prop.getNamespaceStd(), default_value.toStdString());

  QString set_value("Another value to test");
  prop.setString(set_value);
  EXPECT_EQ(prop.getNamespace(), set_value);
  EXPECT_EQ(prop.getNamespaceStd(), set_value.toStdString());
}

TEST(InteractiveMarkerNamespacePropertyTest, is_empty)
{
  rviz_default_plugins::displays::InteractiveMarkerNamespaceProperty prop("Test Name", "");
  EXPECT_TRUE(prop.isEmpty());
  QString set_value("Test non-empty value");
  prop.setString(set_value);
  EXPECT_FALSE(prop.isEmpty());
}
