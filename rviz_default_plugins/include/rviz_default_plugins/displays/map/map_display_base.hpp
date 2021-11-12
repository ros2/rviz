/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * Copyright (c) 2021, Thomas Wodtko @ Institute of Measurement, Control and Microtechnology.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__MAP_DISPLAY_BASE_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__MAP_DISPLAY_BASE_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#ifndef Q_MOC_RUN

#include <OgreMaterial.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTexture.h>
#include <OgreVector3.h>

#endif  // Q_MOC_RUN

#include "rclcpp/time.hpp"

#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/objects/grid.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/validate_floats.hpp"

#include "rviz_default_plugins/displays/map/palette_builder.hpp"
#include "rviz_default_plugins/displays/map/swatch_base.hpp"
#include "rviz_default_plugins/displays/map/q_map_display_object.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre
{
class ManualObject;
}

namespace rviz_default_plugins
{
namespace displays
{
/**
 * \class MapDisplayBase
 * \brief Base class which means to display data using swatches.
 *        Message/Data type is templated,
 *        which let message specific displays inherit the functionality
 */
template<class MessageT, class UpdateMessageT = MessageT>
class RVIZ_DEFAULT_PLUGINS_PUBLIC MapDisplayBase : public
  rviz_common::MessageFilterDisplay<MessageT>
{
public:
  // simplify usage of MFDClass
  using MFDClass = typename rviz_common::MessageFilterDisplay<MessageT>::MFDClass;

  // simplify usage of message ptrs
  using MsgConstSharedPtr = typename MessageT::ConstSharedPtr;
  using UpdateMsgConstSharedPtr = typename UpdateMessageT::ConstSharedPtr;

  // TODO(botteroa-si): Constructor for testing, remove once ros_nodes can be mocked and call
  // initialize() instead
  /**
   * @brief given a context, the display is hooked into the scene
   * @param context
   */
  explicit MapDisplayBase(rviz_common::DisplayContext * context);
  /** @brief set up qt related member */
  MapDisplayBase();
  /** @brief clear swatches and drop subscriptions */
  ~MapDisplayBase() override;

  /** @brief initialize ros specific member */
  void onInitialize() override;
  /** @brief transform map to new fixed frame */
  void fixedFrameChanged() override;
  /** @brief clear swatches, keep subscriptions active */
  void reset() override;

  // simple getter functions
  float getResolution() {return resolution_;}
  size_t getWidth() {return width_;}
  size_t getHeight() {return height_;}

  /** @brief Copy msg into current_map_ and call showMap(). */
  void processMessage(typename MessageT::ConstSharedPtr msg) override;

  // the following functions are called by the helper q_map_display_object
  // whenever parameter are updated and respective SLOTS are triggered
  /** @brief check map validity and call showValidMap() if applicable*/
  void showMap();

protected:
  /** @brief update alpha on all swatches */
  virtual void updateAlpha();
  /** @brief update draw under on all swatches */
  void updateDrawUnder() const;
  /** @brief Show current_map_ in the scene. */
  void transformMap();
  /** @brief update update topic subscription */
  void updateMapUpdateTopic();

  // communication related functions
  /** @brief update property and call super class function */
  void updateTopic() override;
  /** @brief call transformMap() */
  void update(float wall_dt, float ros_dt) override;

  /** @brief update property and call super class function */
  void subscribe() override;
  /** @brief call super class function and update property */
  void unsubscribe() override;

  /** @brief call super class function and update property */
  void onEnable() override;

  /** @brief Copy update's data into current_map_ and call showMap(). */
  virtual void incomingUpdate(typename UpdateMessageT::ConstSharedPtr update) = 0;

  /** @brief clear swatches and update property*/
  void clear();

  /** @brief subscribe to update topic using qos_profile from property */
  void subscribeToUpdateTopic();
  /** @brief unsubscribe to update topic */
  void unsubscribeToUpdateTopic();

  // functions to handle swatches and show the current map
  /** @brief create (if necessary) and update swatches update properties
   *         and call transformMap() and updateDrawUnder()*/
  virtual void showValidMap();
  /** @brief if width,height, or resolution differs create new swatches*/
  virtual void resetSwatchesIfNecessary(size_t width, size_t height, float resolution);
  /** @brief create new swatches for current width, height, and resolution */
  virtual void createSwatches();
  /** @brief update values for twice as many swatches*/
  virtual void doubleSwatchNumber(
    size_t & swatch_width, size_t & swatch_height,
    int & number_swatches) const;
  /** @brief create a single swatch,
   *         done by child class since specific swatch class must be used*/
  virtual std::shared_ptr<SwatchBase<MessageT>> createSwatch(
    Ogre::SceneManager * scene_manager,
    Ogre::SceneNode * parent_scene_node,
    size_t x, size_t y, size_t width, size_t height,
    float resolution, bool draw_under
  ) = 0;
  /** @brief try to create as many swatches as needed */
  virtual void tryCreateSwatches(
    size_t width,
    size_t height,
    float resolution,
    size_t swatch_width,
    size_t swatch_height,
    int number_swatches);
  /** @brief determin effective dimension for even and odd number of swatches */
  virtual size_t getEffectiveDimension(
    size_t map_dimension,
    size_t swatch_dimension,
    size_t position
  );

  /** @brief for each swatch, call updateData and set texture*/
  virtual void updateSwatches() const;

  /** @brief validate floats for specific message, done by child class*/
  virtual bool validateFloats(const MessageT & msg) const = 0;

  // protected members
  std::vector<std::shared_ptr<SwatchBase<MessageT>>> swatches_;
  bool loaded_;

  float resolution_;
  size_t width_;
  size_t height_;
  std::string frame_;
  MessageT current_map_;

  typename rclcpp::Subscription<UpdateMessageT>::SharedPtr update_subscription_;
  uint32_t update_messages_received_;

  // helper object to handle qt related interaction via SIGNALS and SLOTS
  QMapDisplayObject * q_helper_object_{nullptr};
};

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::processMessage(
  typename MessageT::ConstSharedPtr msg)
{
  current_map_ = *msg;
  loaded_ = true;
  // updated via signal in case ros spinner is in a different thread
  Q_EMIT q_helper_object_->mapUpdated();
}

template<class MessageT, class UpdateMessageT>
MapDisplayBase<MessageT, UpdateMessageT>::MapDisplayBase()
: loaded_(false),
  resolution_(0.0f),
  width_(0),
  height_(0),
  update_messages_received_(0)
{
  q_helper_object_ = new QMapDisplayObject{this};

  q_helper_object_->showMap_ = [this]() {showMap();};
  q_helper_object_->updateAlpha_ = [this]() {updateAlpha();};
  q_helper_object_->updateDrawUnder_ = [this]() {updateDrawUnder();};
  q_helper_object_->transformMap_ = [this]() {transformMap();};
  q_helper_object_->updateMapUpdateTopic_ = [this]() {updateMapUpdateTopic();};
}

template<class MessageT, class UpdateMessageT>
MapDisplayBase<MessageT, UpdateMessageT>::MapDisplayBase(rviz_common::DisplayContext * context)
: MapDisplayBase<MessageT, UpdateMessageT>()
{
  this->context_ = context;
  this->scene_manager_ = this->context_->getSceneManager();
  this->scene_node_ = this->scene_manager_->getRootSceneNode()->createChildSceneNode();
}

template<class MessageT, class UpdateMessageT>
MapDisplayBase<MessageT, UpdateMessageT>::~MapDisplayBase()
{
  unsubscribe();
  clear();
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::onInitialize()
{
  MFDClass::onInitialize();

  this->rviz_ros_node_ = this->context_->getRosNodeAbstraction();
  q_helper_object_->update_topic_property_->initialize(this->rviz_ros_node_);

  q_helper_object_->update_profile_property_->initialize(
    [this](rclcpp::QoS profile) {
      q_helper_object_->update_profile_ = profile;
      updateMapUpdateTopic();
    });
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::updateTopic()
{
  q_helper_object_->update_topic_property_->setValue(
    this->topic_property_->getTopic() + "_updates");
  MFDClass::updateTopic();
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::subscribe()
{
  if (!this->isEnabled()) {
    return;
  }

  if (this->topic_property_->isEmpty()) {
    this->setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Topic",
      QString("Error subscribing: Empty topic name"));
    return;
  }

  MFDClass::subscribe();

  subscribeToUpdateTopic();
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::subscribeToUpdateTopic()
{
  try {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.event_callbacks.message_lost_callback =
      [&](rclcpp::QOSMessageLostInfo & info)
      {
        std::ostringstream sstm;
        sstm << "Some messages were lost:\n>\tNumber of new lost messages: " <<
          info.total_count_change << " \n>\tTotal number of messages lost: " <<
          info.total_count;
        this->setStatus(
          rviz_common::properties::StatusProperty::Warn, "Update Topic",
          QString(sstm.str().c_str()));
      };

    update_subscription_ =
      this->rviz_ros_node_.lock()->get_raw_node()->
      template create_subscription<UpdateMessageT>(
      q_helper_object_->update_topic_property_->getTopicStd(),
      q_helper_object_->update_profile_,
      [this](const typename UpdateMessageT::ConstSharedPtr message) {
        incomingUpdate(message);
      },
      sub_opts);
    this->setStatus(rviz_common::properties::StatusProperty::Ok, "Update Topic", "OK");
  } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
    this->setStatus(
      rviz_common::properties::StatusProperty::Error, "Update Topic",
      QString("Error subscribing: ") + e.what());
  }
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::unsubscribe()
{
  MFDClass::unsubscribe();
  unsubscribeToUpdateTopic();
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::unsubscribeToUpdateTopic()
{
  update_subscription_.reset();
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::updateAlpha()
{
  float alpha = q_helper_object_->alpha_property_->getFloat();
  Ogre::SceneBlendType scene_blending;
  bool depth_write;

  rviz_rendering::MaterialManager::enableAlphaBlending(scene_blending, depth_write, alpha);

  for (const auto & swatch : swatches_) {
    swatch->updateAlpha(scene_blending, depth_write, alpha);
  }
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::updateDrawUnder() const
{
  bool draw_under = q_helper_object_->draw_under_property_->getValue().toBool();

  if (q_helper_object_->alpha_property_->getFloat() >= rviz_rendering::unit_alpha_threshold) {
    for (const auto & swatch : swatches_) {
      swatch->setDepthWriteEnabled(!draw_under);
    }
  }

  uint8_t group = draw_under ? Ogre::RENDER_QUEUE_4 : Ogre::RENDER_QUEUE_MAIN;
  for (const auto & swatch : swatches_) {
    swatch->setRenderQueueGroup(group);
  }
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::clear()
{
  if (this->isEnabled()) {
    this->setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No map received");
  }

  if (!loaded_) {
    return;
  }

  swatches_.clear();
  height_ = 0;
  width_ = 0;
  resolution_ = 0.0f;

  loaded_ = false;
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::createSwatches()
{
  size_t width = current_map_.info.width;
  size_t height = current_map_.info.height;
  float resolution = current_map_.info.resolution;

  size_t swatch_width = width;
  size_t swatch_height = height;
  int number_swatches = 1;
  // One swatch can have up to 2^16 * 2^16 pixel (8 bit texture, i.e. 4GB of data)
  // Since the width and height are separately limited by 2^16 it might be necessary to have several
  // pieces, however more than 8 swatches is probably unnecessary due to memory limitations
  const size_t maximum_number_swatch_splittings = 4;

  for (size_t i = 0; i < maximum_number_swatch_splittings; ++i) {
    RVIZ_COMMON_LOG_INFO_STREAM(
      "Trying to create a map of size " << width <<
        " x " << height << " using " << number_swatches << " swatches");

    swatches_.clear();
    try {
      tryCreateSwatches(width, height, resolution, swatch_width, swatch_height, number_swatches);
      updateDrawUnder();
      return;
    } catch (Ogre::InvalidParametersException &) {
      doubleSwatchNumber(swatch_width, swatch_height, number_swatches);
    } catch (Ogre::RenderingAPIException &) {
      // This exception seems no longer thrown on some systems. May still be relevant for others.
      doubleSwatchNumber(swatch_width, swatch_height, number_swatches);
    }
  }
  RVIZ_COMMON_LOG_ERROR_STREAM(
    "Creating " << number_swatches <<
      "failed. This map is too large to be displayed by RViz.");
  swatches_.clear();
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::doubleSwatchNumber(
  size_t & swatch_width, size_t & swatch_height, int & number_swatches) const
{
  RVIZ_COMMON_LOG_ERROR_STREAM(
    "Failed to create map using " << number_swatches <<
      " swatches. At least one swatch seems to need too much memory");

  if (swatch_width > swatch_height) {
    swatch_width /= 2;
  } else {
    swatch_height /= 2;
  }
  number_swatches *= 2;
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::tryCreateSwatches(
  size_t width,
  size_t height,
  float resolution,
  size_t swatch_width,
  size_t swatch_height,
  int number_swatches)
{
  size_t x = 0;
  size_t y = 0;
  for (int i = 0; i < number_swatches; i++) {
    size_t effective_width = getEffectiveDimension(width, swatch_width, x);
    size_t effective_height = getEffectiveDimension(height, swatch_height, y);

    swatches_.push_back(
      createSwatch(
        this->scene_manager_,
        this->scene_node_,
        x, y,
        effective_width,
        effective_height,
        resolution,
        q_helper_object_->draw_under_property_->getValue().toBool()));

    swatches_[i]->updateData(current_map_);

    x += effective_width;
    if (x >= width) {
      x = 0;
      y += effective_height;
    }
  }
  updateAlpha();
}

template<class MessageT, class UpdateMessageT>
size_t MapDisplayBase<MessageT, UpdateMessageT>::getEffectiveDimension(
  size_t map_dimension, size_t swatch_dimension, size_t position)
{
  // Last swatch is bigger than swatch_dimension for odd numbers.
  // subtracting the swatch_dimension in the LHS handles this case.
  return map_dimension - position - swatch_dimension >= swatch_dimension ?
         swatch_dimension :
         map_dimension - position;
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::showMap()
{
  if (current_map_.data.empty()) {
    return;
  }

  if (!validateFloats(current_map_)) {
    this->setStatus(
      rviz_common::properties::StatusProperty::Error, "Map",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  size_t width = current_map_.info.width;
  size_t height = current_map_.info.height;

  if (width * height == 0) {
    std::string message =
      "Map is zero-sized (" + std::to_string(width) + "x" + std::to_string(height) + ")";
    this->setStatus(
      rviz_common::properties::StatusProperty::Error, "Map", QString::fromStdString(message));
    return;
  }

  if (width * height != current_map_.data.size()) {
    std::string message =
      "Data size doesn't match width*height: width = " + std::to_string(width) + ", height = " +
      std::to_string(height) + ", data size = " + std::to_string(current_map_.data.size());
    this->setStatus(
      rviz_common::properties::StatusProperty::Error, "Map", QString::fromStdString(message));
    return;
  }

  this->setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Map received");

  RVIZ_COMMON_LOG_DEBUG_STREAM(
    "Received a " << current_map_.info.width <<
      " X " << current_map_.info.height <<
      " map @ " << current_map_.info.resolution << "m/pix\n");

  showValidMap();
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::showValidMap()
{
  size_t width = current_map_.info.width;
  size_t height = current_map_.info.height;

  float resolution = current_map_.info.resolution;

  resetSwatchesIfNecessary(width, height, resolution);

  frame_ = current_map_.header.frame_id;
  if (frame_.empty()) {
    frame_ = "/map";
  }

  updateSwatches();

  this->setStatus(rviz_common::properties::StatusProperty::Ok, "Map", "Map OK");

  q_helper_object_->resolution_property_->setValue(resolution);
  q_helper_object_->width_property_->setValue(static_cast<unsigned int>(width));
  q_helper_object_->height_property_->setValue(static_cast<unsigned int>(height));

  q_helper_object_->position_property_->setVector(
    rviz_common::pointMsgToOgre(current_map_.info.origin.position));
  q_helper_object_->orientation_property_->setQuaternion(
    rviz_common::quaternionMsgToOgre(current_map_.info.origin.orientation));

  transformMap();

  updateDrawUnder();

  this->context_->queueRender();
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::resetSwatchesIfNecessary(
  size_t width, size_t height, float resolution
)
{
  if (width != width_ || height != height_ || resolution_ != resolution) {
    createSwatches();
    width_ = width;
    height_ = height;
    resolution_ = resolution;
  }
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::updateSwatches() const
{
  for (const auto & swatch : swatches_) {
    swatch->updateData(current_map_);

    Ogre::Pass * pass = swatch->getTechniquePass();
    Ogre::TextureUnitState * tex_unit = nullptr;
    if (pass->getNumTextureUnitStates() > 0) {
      tex_unit = pass->getTextureUnitState(0);
    } else {
      tex_unit = pass->createTextureUnitState();
    }

    tex_unit->setTextureName(swatch->getTextureName());
    tex_unit->setTextureFiltering(Ogre::TFO_NONE);
    swatch->setVisible(true);
    swatch->resetOldTexture();
  }
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::transformMap()
{
  if (!loaded_) {
    return;
  }

  rclcpp::Time transform_time = this->context_->getClock()->now();

  if (q_helper_object_->transform_timestamp_property_->getBool()) {
    transform_time = current_map_.header.stamp;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!this->context_->getFrameManager()->transform(
      frame_, transform_time, current_map_.info.origin, position, orientation) &&
    !this->context_->getFrameManager()->transform(
      frame_, rclcpp::Time(0, 0, this->context_->getClock()->get_clock_type()),
      current_map_.info.origin, position, orientation))
  {
    this->setMissingTransformToFixedFrame(frame_);
    this->scene_node_->setVisible(false);
  } else {
    this->setTransformOk();

    this->scene_node_->setPosition(position);
    this->scene_node_->setOrientation(orientation);
  }
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::fixedFrameChanged()
{
  transformMap();
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::reset()
{
  MFDClass::reset();
  update_messages_received_ = 0;
  clear();
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;

  transformMap();
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::onEnable()
{
  MFDClass::onEnable();
  this->setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No map received");
}

template<class MessageT, class UpdateMessageT>
void MapDisplayBase<MessageT, UpdateMessageT>::updateMapUpdateTopic()
{
  unsubscribeToUpdateTopic();
  reset();
  subscribeToUpdateTopic();
  this->context_->queueRender();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__MAP_DISPLAY_BASE_HPP_
