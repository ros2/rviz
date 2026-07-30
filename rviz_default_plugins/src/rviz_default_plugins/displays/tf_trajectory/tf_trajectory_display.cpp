// Copyright (c) 2015, JSK Lab
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

// This file is a ROS 2 port of jsk_rviz_plugins TFTrajectoryDisplay.

#include "rviz_default_plugins/displays/tf_trajectory/tf_trajectory_display.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include "rclcpp/duration.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/properties/tf_frame_property.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"

namespace rviz_default_plugins
{
namespace displays
{

namespace
{

constexpr uint32_t kMaxElementsPerLine = 65536U / 4U;

// Upper bound for the Duration property.  rclcpp::Duration::from_seconds()
// multiplies by 1e9 into an int64, so an unbounded float overflows it.
constexpr float kMaxDurationSeconds = 3600.0f;

// Backstop against unbounded growth if duration-based trimming is ever unable
// to make progress.  At 60 Hz this is over four hours of history.
constexpr size_t kMaxTrajectoryPoints = 1000000U;

}  // namespace

TFTrajectoryDisplay::TFTrajectoryDisplay()
: Display(),
  frame_property_(nullptr),
  duration_property_(nullptr),
  line_width_property_(nullptr),
  color_property_(nullptr)
{
  frame_property_ = new rviz_common::properties::TfFrameProperty(
    "Frame", "",
    "TF frame to visualize trajectory.",
    this, nullptr, false, SLOT(updateFrame()), this);

  duration_property_ = new rviz_common::properties::FloatProperty(
    "Duration", 10.0f,
    "Duration in seconds to keep trajectory history.",
    this, SLOT(updateDuration()), this);
  duration_property_->setMin(0.0f);
  duration_property_->setMax(kMaxDurationSeconds);

  line_width_property_ = new rviz_common::properties::FloatProperty(
    "Line Width", 0.05f,
    "Trajectory line width in meters.",
    this, SLOT(updateLineWidth()), this);
  line_width_property_->setMin(0.0f);

  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(25, 255, 240),
    "Color of the trajectory line.",
    this, SLOT(updateColor()), this);
}

TFTrajectoryDisplay::TFTrajectoryDisplay(rviz_common::DisplayContext * context)
: TFTrajectoryDisplay()
{
  context_ = context;
  scene_manager_ = context_->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  frame_property_->setFrameManager(context_->getFrameManager());
  trajectory_line_ = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
}

TFTrajectoryDisplay::~TFTrajectoryDisplay() = default;

void TFTrajectoryDisplay::onInitialize()
{
  frame_property_->setFrameManager(context_->getFrameManager());
  trajectory_line_ = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);

  updateFrame();
  updateDuration();
  updateLineWidth();
  updateColor();
}

void TFTrajectoryDisplay::onEnable()
{
  clearTrajectory();
}

void TFTrajectoryDisplay::onDisable()
{
  clearTrajectory();
}

void TFTrajectoryDisplay::fixedFrameChanged()
{
  clearTrajectory();
}

void TFTrajectoryDisplay::reset()
{
  Display::reset();
  clearTrajectory();
}

void TFTrajectoryDisplay::update(std::chrono::nanoseconds wall_dt, std::chrono::nanoseconds ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;

  if (!trajectory_line_) {
    return;
  }

  if (frame_id_.empty()) {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Transform", "Frame is empty");
    return;
  }

  const rclcpp::Time now = context_->getClock()->now();

  // A clock-source switch (use_sim_time toggled) or a jump backwards in time
  // (a rosbag looping) invalidates every stamp recorded so far: rclcpp::Time
  // refuses to subtract across clock types, and points stamped in the future
  // would outlive Duration until time caught up again.
  if (!trajectory_.empty() &&
    (trajectory_.back().stamp.get_clock_type() != now.get_clock_type() ||
    now < trajectory_.back().stamp))
  {
    clearTrajectory();
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  // Ask for the latest available transform, as Axes does. Requesting it at
  // `now` makes tf2 resolve a timestamp newer than the most recent sample, so
  // the lookup fails by extrapolation on every frame. `now` is only used to
  // stamp the sample for duration trimming.
  const bool have_transform =
    context_->getFrameManager()->getTransform(frame_id_, position, orientation);

  bool appended = false;
  if (have_transform) {
    setTransformOk();
    // With a paused clock (`/clock` stopped under use_sim_time) `now` never
    // advances, so trimming can never drop anything.  Appending one sample per
    // render tick would then grow the history without bound; only record a
    // sample once the timestamp has actually moved forward.
    if (trajectory_.empty() || now > trajectory_.back().stamp) {
      trajectory_.push_back(TrajectoryPoint{
            now,
            position.x,
            position.y,
            position.z
          });
      appended = true;
    }
  } else {
    std::string error;
    if (context_->getFrameManager()->transformHasProblems(frame_id_, error)) {
      setStatusStd(rviz_common::properties::StatusProperty::Error, "Transform", error);
    } else {
      setMissingTransformToFixedFrame(frame_id_);
    }
  }

  // Trim on every tick, including the path where the lookup failed: once the
  // frame stops being published the history must still expire after Duration
  // rather than staying on screen forever.
  const size_t size_before_trim = trajectory_.size();
  trimExpiredPoints(now);

  if (appended || trajectory_.size() != size_before_trim) {
    redrawTrajectory();
    context_->queueRender();
  }
}

void TFTrajectoryDisplay::updateFrame()
{
  frame_id_ = frame_property_->getFrameStd();
  clearTrajectory();
}

void TFTrajectoryDisplay::updateDuration()
{
  if (!context_ || trajectory_.empty()) {
    return;
  }

  trimExpiredPoints(context_->getClock()->now());
  redrawTrajectory();
  context_->queueRender();
}

void TFTrajectoryDisplay::updateColor()
{
  if (!trajectory_line_) {
    return;
  }

  const QColor color = color_property_->getColor();
  trajectory_line_->setColor(color.redF(), color.greenF(), color.blueF(), 1.0f);
  context_->queueRender();
}

void TFTrajectoryDisplay::updateLineWidth()
{
  if (!trajectory_line_) {
    return;
  }

  trajectory_line_->setLineWidth(line_width_property_->getFloat());
  context_->queueRender();
}

void TFTrajectoryDisplay::clearTrajectory()
{
  trajectory_.clear();
  if (trajectory_line_) {
    trajectory_line_->clear();
  }
  if (context_) {
    context_->queueRender();
  }
}

void TFTrajectoryDisplay::trimExpiredPoints(const rclcpp::Time & now)
{
  const rclcpp::Duration keep_duration = rclcpp::Duration::from_seconds(
    std::clamp(duration_property_->getFloat(), 0.0f, kMaxDurationSeconds));
  while (!trajectory_.empty() && now - trajectory_.front().stamp > keep_duration) {
    trajectory_.pop_front();
  }
  while (trajectory_.size() > kMaxTrajectoryPoints) {
    trajectory_.pop_front();
  }
}

void TFTrajectoryDisplay::redrawTrajectory()
{
  if (!trajectory_line_) {
    return;
  }

  trajectory_line_->clear();
  if (trajectory_.empty()) {
    return;
  }

  const auto point_count = static_cast<uint32_t>(trajectory_.size());
  trajectory_line_->setNumLines((point_count - 1U) / kMaxElementsPerLine + 1U);
  trajectory_line_->setMaxPointsPerLine(std::min(point_count, kMaxElementsPerLine));
  trajectory_line_->setLineWidth(line_width_property_->getFloat());

  const QColor color = color_property_->getColor();
  trajectory_line_->setColor(color.redF(), color.greenF(), color.blueF(), 1.0f);
  for (const auto & point : trajectory_) {
    trajectory_line_->addPoint(Ogre::Vector3(point.x, point.y, point.z));
  }
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::TFTrajectoryDisplay, rviz_common::Display)
