/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <memory>

#include <OgreSceneNode.h>

// TODO(Martin-Idel-SI): revisit when message filter is working again
// #include <tf/transform_listener.h>

#include "./point_cloud_common.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager.hpp"
#include "rviz_common/properties/int_property.hpp"

#include "point_cloud_display.hpp"

namespace rviz_default_plugins
{

PointCloudDisplay::PointCloudDisplay()
: point_cloud_common_(new PointCloudCommon(this))
{
  queue_size_property_ = new rviz_common::properties::IntProperty("Queue Size", 10,
      "Advanced: set the size of the incoming PointCloud message queue. "
      " Increasing this is useful if your incoming TF data is delayed significantly "
      "from your PointCloud data, but it can greatly increase memory usage if the "
      "messages are big.",
      this, SLOT(updateQueueSize()));
}

PointCloudDisplay::~PointCloudDisplay()
{
  delete point_cloud_common_;
}

void PointCloudDisplay::onInitialize()
{
  MFDClass::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
}

void PointCloudDisplay::updateQueueSize()
{
  // TODO(Martin-Idel-SI): revisit when message filter is working again
//  tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
}

void PointCloudDisplay::processMessage(const sensor_msgs::msg::PointCloud::ConstSharedPtr & cloud)
{
  point_cloud_common_->addMessage(cloud);
}

void PointCloudDisplay::update(float wall_dt, float ros_dt)
{
  point_cloud_common_->update(wall_dt, ros_dt);
}

void PointCloudDisplay::reset()
{
  MFDClass::reset();
  point_cloud_common_->reset();
}

}  // namespace rviz_default_plugins

// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS( rviz::PointCloudDisplay, rviz::Display )
