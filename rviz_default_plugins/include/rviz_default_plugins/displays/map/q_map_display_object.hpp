/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__Q_MAP_DISPLAY_OBJECT_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__Q_MAP_DISPLAY_OBJECT_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"

#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/vector_property.hpp"

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here

namespace rviz_default_plugins
{
namespace displays
{
/**
 * @class Helper class to handle all qt related communication for the MapDisplayBase class.
 *        Since template classes cannot by Q_OBJECTS, the functionality has to be separated
 */
class QMapDisplayObject : public QObject
{
  Q_OBJECT

public:
  // naming the callback function type
  using EmptyCallback = std::function<void ()>;

  /** @brief set up all properties and hook them into the parent branch */
  explicit QMapDisplayObject(rviz_common::properties::Property * parent);
  ~QMapDisplayObject() override = default;

public Q_SLOTS:
  /** @brief connected to mapUpdated, call showMap_() is valid */
  void showMap();

  // the following functions are triggered by property updates
  // respective callbacks can be set and are called if valid
  /** @brief call updateAlpha_() if valid */
  virtual void updateAlpha();
  /** @brief call updateDrawUnder_() if valid */
  void updateDrawUnder();
  /** @brief call transformMap_() if valid */
  void transformMap();
  void updateMapUpdateTopic();

Q_SIGNALS:
  /** @brief Emitted when a new map is received */
  void mapUpdated();

public:
  // "SLOTS" to use non O_OBJECT functions
  EmptyCallback showMap_;
  EmptyCallback updateAlpha_;
  EmptyCallback updateDrawUnder_;
  EmptyCallback transformMap_;
  EmptyCallback updateMapUpdateTopic_;

  rclcpp::QoS update_profile_;

  // properties handled by this helper class
  rviz_common::properties::RosTopicProperty * update_topic_property_ {nullptr};
  rviz_common::properties::QosProfileProperty * update_profile_property_ {nullptr};
  rviz_common::properties::FloatProperty * resolution_property_ {nullptr};
  rviz_common::properties::IntProperty * width_property_ {nullptr};
  rviz_common::properties::IntProperty * height_property_ {nullptr};
  rviz_common::properties::VectorProperty * position_property_ {nullptr};
  rviz_common::properties::QuaternionProperty * orientation_property_ {nullptr};
  rviz_common::properties::FloatProperty * alpha_property_ {nullptr};
  rviz_common::properties::Property * draw_under_property_ {nullptr};
  rviz_common::properties::BoolProperty * transform_timestamp_property_ {nullptr};
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__Q_MAP_DISPLAY_OBJECT_HPP_
