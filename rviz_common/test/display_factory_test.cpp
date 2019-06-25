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

#include <gmock/gmock.h>

#include <memory>
#include <utility>

#include <QString>  // NOLINT: cpplint cannot handle include order here

#include "../src/rviz_common/display_factory.hpp"

using namespace ::testing;  // NOLINT use namespace testing to not clutter tests

class TestDisplayFactory : public rviz_common::DisplayFactory
{
public:
  explicit TestDisplayFactory(QString path_suffix = "/test/resources/plugins_description.xml")
  : rviz_common::DisplayFactory(), path_suffix_(std::move(path_suffix))
  {}

  // overwrite getPluginManifestPath to get our own test resource. This method is called in the
  // real method we want to test which uses tinyxml to parse the plugin manifest.
  QString getPluginManifestPath(const QString & class_id) const override
  {
    (void) class_id;
    const auto path_prefix = QString(_TEST_PLUGIN_DESCRIPTIONS);
    return path_prefix + path_suffix_;
  }

private:
  QString path_suffix_;
};

TEST(TestDisplayFactory, getMessageTypes_finds_sensor_msgs_for_fake_camera_display) {
  auto display_factory = std::make_unique<TestDisplayFactory>();
  const QSet<QString> message_types = display_factory->getMessageTypes("rviz_common_test/Camera");
  ASSERT_THAT(message_types, Contains("sensor_msgs/msg/Image"));
  ASSERT_THAT(message_types, Contains("sensor_msgs/msg/CompressedImage"));
}

TEST(TestDisplayFactory, getMessageTypes_finds_no_messages_for_fake_grid_display) {
  auto display_factory = std::make_unique<TestDisplayFactory>();
  const QSet<QString> message_types = display_factory->getMessageTypes("rviz_common_test/Grid");
  ASSERT_THAT(message_types, IsEmpty());
}

TEST(TestDisplayFactory, getMessageTypes_finds_no_messages_for_missing_display) {
  auto display_factory = std::make_unique<TestDisplayFactory>();
  const QSet<QString> message_types = display_factory->getMessageTypes(
    "rviz_common_test/MissingDisplay");
  ASSERT_THAT(message_types, IsEmpty());
}

TEST(TestDisplayFactory, getMessageTypes_finds_no_messages_for_xml_with_wrong_root) {
  auto display_factory =
    std::make_unique<TestDisplayFactory>("/test/resources/broken_plugins.xml");
  const QSet<QString> message_types = display_factory->getMessageTypes("rviz_common_test/Camera");
  ASSERT_THAT(message_types, IsEmpty());
}
