/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <gtest/gtest.h>

#include <QApplication>

#include <sstream>
#include <string>

#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/yaml_config_reader.hpp"
#include "rviz_common/yaml_config_writer.hpp"

#include "mock_display.hpp"
#include "mock_display_group.hpp"

using namespace rviz_common;  // NOLINT

TEST(Display, load_properties) {
  std::stringstream input(
    "Name: sample\n"
    "Enabled: true\n"
    "Count: 7\n"
    "Number: 3.2\n"
    "Offset: {X: -1, Y: 1.1, Z: 1.1e3}\n"
    "Color: white\n"
    "Style: loosey goosey\n");

  YamlConfigReader reader;
  Config config;
  reader.readStream(config, input);

  MockDisplay d;
  d.load(config);

  EXPECT_EQ(7, d.count_->getValue().toInt());
  EXPECT_EQ("loosey goosey", d.style_->getValue().toString().toStdString());
  EXPECT_EQ(3.2f, d.number_->getValue().toFloat());
  Ogre::Vector3 offset = d.offset_->getVector();
  EXPECT_EQ(-1.f, offset.x);
  EXPECT_EQ(1.1f, offset.y);
  EXPECT_EQ(1100.f, offset.z);
  EXPECT_EQ("255; 255; 255", d.color_->getValue().toString().toStdString());
  EXPECT_TRUE(d.getValue().toBool());
}

TEST(DisplayGroup, load_properties) {
  std::stringstream input(
    "Name: root\n"
    "Enabled: true\n"
    "Displays:\n"
    " -\n"
    "   Class: MockDisplay\n"
    "   Name: Steven\n"
    "   Enabled: false\n"
    "   Count: 17\n"
    " -\n"
    "   Name: sub group\n"
    "   Class: DisplayGroup\n"
    "   Enabled: true\n"
    "   Displays:\n"
    "    -\n"
    "      Class: MockDisplay\n"
    "      Name: Curly\n"
    "      Enabled: false\n"
    "      Count: 900\n"
  );

  YamlConfigReader reader;
  Config config;
  reader.readStream(config, input);

  MockDisplayGroup g;
  g.load(config);

  EXPECT_TRUE(g.getValue().toBool());
  EXPECT_FALSE(g.subProp("Steven")->getValue().toBool());
  EXPECT_EQ(17, g.subProp("Steven")->subProp("Count")->getValue().toInt());
  EXPECT_EQ(900, g.subProp("sub group")->subProp("Curly")->subProp("Count")->getValue().toInt());
}

TEST(Display, save_properties) {
  MockDisplay d;
  d.setName("Steven");
  d.subProp("Count")->setValue(37);

  YamlConfigWriter writer;
  Config config;
  d.save(config);
  QString out = writer.writeString(config);

  // Since we instantiated the display directly instead of using the
  // DisplayFactory, it won't know its class name.
  EXPECT_EQ(
    std::string(
      "Class: \"\"\n"
      "Color: 10; 20; 30\n"
      "Count: 37\n"
      "Enabled: false\n"
      "Name: Steven\n"
      "Number: 1.5\n"
      "Offset:\n"
      "  X: 1\n"
      "  Y: 2\n"
      "  Z: 3\n"
      "Style: chunky\n"
      "Value: false\n"

    ), out.toStdString());
}

TEST(DisplayGroup, save_properties) {
  MockDisplayGroup g;
  g.setName("Charles");

  auto d = new MockDisplay;
  d->setName("Steven");
  d->subProp("Count")->setValue(101);
  g.addChild(d);

  d = new MockDisplay;
  d->setName("Katherine");
  d->subProp("Number")->setValue(0.125);
  g.addChild(d);

  YamlConfigWriter writer;
  Config config;
  g.save(config);
  QString out = writer.writeString(config);

  // Since we instantiated the display directly instead of using the
  // DisplayFactory, it won't know its class name.
  EXPECT_EQ(
    std::string(
      "Class: \"\"\n"
      "Displays:\n"
      "  - Class: \"\"\n"
      "    Color: 10; 20; 30\n"
      "    Count: 101\n"
      "    Enabled: false\n"
      "    Name: Steven\n"
      "    Number: 1.5\n"
      "    Offset:\n"
      "      X: 1\n"
      "      Y: 2\n"
      "      Z: 3\n"
      "    Style: chunky\n"
      "    Value: false\n"
      "  - Class: \"\"\n"
      "    Color: 10; 20; 30\n"
      "    Count: 10\n"
      "    Enabled: false\n"
      "    Name: Katherine\n"
      "    Number: 0.125\n"
      "    Offset:\n"
      "      X: 1\n"
      "      Y: 2\n"
      "      Z: 3\n"
      "    Style: chunky\n"
      "    Value: false\n"
      "Enabled: false\n"
      "Name: Charles\n"
    ), out.toStdString());
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
