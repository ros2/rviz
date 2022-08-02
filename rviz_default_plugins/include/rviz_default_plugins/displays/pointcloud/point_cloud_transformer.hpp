/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_TRANSFORMER_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_TRANSFORMER_HPP_

#include <vector>

#include <QObject>  // NOLINT

#ifndef Q_MOC_RUN
#include <OgreVector.h>
#include <OgreColourValue.h>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rviz_rendering/objects/point_cloud.hpp"

#include "rviz_default_plugins/visibility_control.hpp"
#endif

namespace Ogre
{
class Matrix4;
}

namespace rviz_common
{
namespace properties
{

class Property;

}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{

typedef std::vector<rviz_rendering::PointCloud::Point> V_PointCloudPoint;

class RVIZ_DEFAULT_PLUGINS_PUBLIC PointCloudTransformer : public QObject
{
  Q_OBJECT

public:
  virtual void init() {}

  /**
   * \brief Enumeration of support levels.  Basic levels (Support_None, Support_XYZ, Support_Color) can be
   * ored together to form a mask, Support_Both is provided as a convenience.
   */
  enum SupportLevel
  {
    Support_None = 0,
    Support_XYZ = 1 << 1,
    Support_Color = 1 << 2,
    Support_Both = Support_XYZ | Support_Color,
  };

  /**
   * \brief Returns a level of support for a specific cloud.  This level of support is a mask using the SupportLevel enum.
   */
  virtual uint8_t supports(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud) = 0;
  /**
   * \brief Transforms a PointCloud2 into an rviz::PointCloud.  The rviz::PointCloud is assumed to have been preallocated into the correct
   * size.  The mask determines which part of the cloud should be output (xyz or color).  This method will only be called if supports() of the same
   * cloud has returned a non-zero mask, and will only be called with masks compatible with the one returned from supports()
   */
  virtual bool transform(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    uint32_t mask,
    const Ogre::Matrix4 & transform,
    V_PointCloudPoint & out) = 0;

  /**
   * \brief "Score" a message for how well supported the message is.  For example, a "flat color" transformer can support any cloud, but will
   * return a score of 0 here since it should not be preferred over others that explicitly support fields in the message.  This allows that
   * "flat color" transformer to still be selectable, but generally not chosen automatically.
   */
  virtual uint8_t score(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    (void) cloud;
    return 0;
  }

  /**
   * \brief Create any properties necessary for this transformer.
   * Will be called once when the transformer is loaded.  All
   * properties must be added to the out_props vector.
   */
  virtual void createProperties(
    rviz_common::properties::Property * parent_property,
    uint32_t mask,
    QList<rviz_common::properties::Property *> & out_props)
  {
    (void) parent_property;
    (void) mask;
    (void) out_props;
  }

  /**
 * \brief Hide properties that are currently not in use.
 */
  virtual void hideUnusedProperties() {}

  // class_id and description are required to be used with rviz_common::PluginlibFactory
  QString getClassId() const {return class_id_;}
  QString getDescription() const {return description_;}
  void setClassId(const QString & class_id) {class_id_ = class_id;}
  void setDescription(const QString & description) {description_ = description;}

Q_SIGNALS:
  /** @brief Subclasses should emit this signal whenever they think the points should be re-transformed.
   */
  void needRetransform();

protected:
  // class_id and description are required to be used with rviz_common::PluginlibFactory
  QString class_id_;
  QString description_;
};

}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_TRANSFORMER_HPP_
