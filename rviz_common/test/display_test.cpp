// Copyright (c) 2012, Willow Garage, Inc.
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


#include <gtest/gtest.h>

#include <QApplication>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/properties/display_group_visibility_property.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/yaml_config_reader.hpp"
#include "rviz_common/yaml_config_writer.hpp"

#include "mock_display.hpp"
#include "mock_display_group.hpp"

using namespace rviz_common;  // NOLINT

namespace
{

// Records whether the model starts inserting or removing rows while another
// insertion or removal is still in progress.
class NestedModelChangeDetector
{
public:
  explicit NestedModelChangeDetector(QAbstractItemModel * model)
  {
    auto begin = [this]() {
        if (depth_ > 0) {
          nested_ = true;
        }
        depth_++;
      };
    auto end = [this]() {depth_--;};
    QObject::connect(model, &QAbstractItemModel::rowsAboutToBeInserted, &context_, begin);
    QObject::connect(model, &QAbstractItemModel::rowsInserted, &context_, end);
    QObject::connect(model, &QAbstractItemModel::rowsAboutToBeRemoved, &context_, begin);
    QObject::connect(model, &QAbstractItemModel::rowsRemoved, &context_, end);
  }

  bool nested() const
  {
    return nested_;
  }

private:
  QObject context_;
  int depth_ = 0;
  bool nested_ = false;
};

}  // namespace

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

TEST(DisplayGroup, removeAllDisplays_does_not_nest_model_changes_for_visibility_listeners) {
  auto root = new MockDisplayGroup;
  properties::PropertyTreeModel model(root);

  auto camera = new MockDisplay;
  root->addDisplay(camera);
  root->addDisplay(new MockDisplay);
  root->addDisplay(new MockDisplay);
  // Like a Camera display, which removes its visibility row for every display
  // removed from the group it watches.
  new properties::DisplayGroupVisibilityProperty(
    1, root, camera, "Visibility", true, "", camera);

  NestedModelChangeDetector detector(&model);
  root->removeAllDisplays();

  EXPECT_FALSE(detector.nested());
  EXPECT_EQ(0, root->numDisplays());
}

TEST(DisplayGroup, removeAllDisplays_deletes_nested_groups_outside_the_model_removal) {
  auto root = new MockDisplayGroup;
  properties::PropertyTreeModel model(root);

  auto nested_group = new MockDisplayGroup;
  root->addDisplay(nested_group);
  nested_group->addDisplay(new MockDisplay);
  // A property that outlives the removal and watches the nested group directly.
  auto visibility = new properties::DisplayGroupVisibilityProperty(
    1, nested_group, nullptr, "Visibility", true, "", root);
  ASSERT_EQ(1, visibility->numChildren());

  NestedModelChangeDetector detector(&model);
  root->removeAllDisplays();

  EXPECT_FALSE(detector.nested());
  EXPECT_EQ(0, root->numDisplays());
  EXPECT_EQ(0, visibility->numChildren());
}

TEST(DisplayGroup, displayRemoved_is_emitted_while_the_display_is_still_in_the_group) {
  MockDisplayGroup group;
  auto display = new MockDisplay;
  group.addDisplay(display);

  int num_displays_in_slot = -1;
  QObject context;
  QObject::connect(
    &group, &rviz_common::DisplayGroup::displayRemoved, &context,
    [&](rviz_common::Display * removed) {
      EXPECT_EQ(display, removed);
      num_displays_in_slot = group.numDisplays();
    });
  std::unique_ptr<rviz_common::Display> taken(group.takeDisplay(display));

  EXPECT_EQ(display, taken.get());
  EXPECT_EQ(1, num_displays_in_slot);
  EXPECT_EQ(0, group.numDisplays());
}

TEST(DisplayGroup, takeDisplay_takes_the_right_display_when_a_listener_takes_another_one) {
  MockDisplayGroup group;
  auto first = new MockDisplay;
  auto second = new MockDisplay;
  group.addDisplay(first);
  group.addDisplay(second);

  std::unique_ptr<rviz_common::Display> taken_by_listener;
  QObject context;
  QObject::connect(
    &group, &rviz_common::DisplayGroup::displayRemoved, &context,
    [&](rviz_common::Display * removed) {
      if (removed == second) {
        taken_by_listener.reset(group.takeDisplay(first));
      }
    });
  std::unique_ptr<rviz_common::Display> taken(group.takeDisplay(second));

  EXPECT_EQ(second, taken.get());
  EXPECT_EQ(first, taken_by_listener.get());
  EXPECT_EQ(0, group.numDisplays());
}

TEST(DisplayGroup, removeAllDisplays_signals_each_display_once_when_a_listener_takes_one) {
  MockDisplayGroup group;
  auto first = new MockDisplay;
  auto second = new MockDisplay;
  group.addDisplay(first);
  group.addDisplay(second);

  std::unique_ptr<rviz_common::Display> taken_by_listener;
  std::vector<rviz_common::Display *> removed_displays;
  QObject context;
  QObject::connect(
    &group, &rviz_common::DisplayGroup::displayRemoved, &context,
    [&](rviz_common::Display * removed) {
      removed_displays.push_back(removed);
      if (removed == second) {
        taken_by_listener.reset(group.takeDisplay(first));
      }
    });
  group.removeAllDisplays();

  EXPECT_EQ((std::vector<rviz_common::Display *>{second, first}), removed_displays);
  EXPECT_EQ(first, taken_by_listener.get());
  EXPECT_EQ(0, group.numDisplays());
}
