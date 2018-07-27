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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__ONLY_TF_COMPATIBLE_BASE_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__ONLY_TF_COMPATIBLE_BASE_DISPLAY_HPP_

#include <string>
#include <memory>

#include <QString>  // NOLINT

#include "rviz_common/ros_topic_display.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"
#include "rviz_default_plugins/transformation/tf_wrapper.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace displays
{

/** \brief Convenience base class for displays that can only work with TFTransformer.
 *  It helps handling the change of the transformer.
 */
template<typename MessageType>
class RVIZ_DEFAULT_PLUGINS_PUBLIC OnlyTfCompatibleBaseDisplay : public
  rviz_common::RosTopicDisplay<MessageType>
{
public:
  OnlyTfCompatibleBaseDisplay()
  : using_tf_transformer_(true), display_name_("") {}

  ~OnlyTfCompatibleBaseDisplay() override = default;

  virtual void updateDisplayAccordingToTransformerType()
  {
    using_tf_transformer_ = isTFTransformer(this->context_->getFrameManager()->getInternalPtr());
    this->clearStatuses();
    updatePropertiesVisibility();

    if (!using_tf_transformer_) {
      this->onDisable();
      this->setStatus(
        rviz_common::properties::StatusProperty::Error,
        "Transformer",
        QString::fromStdString(
          display_name_ +
          " display works only with TF: set Transformer to TF to enable the display."));
    }
  }

protected:
  virtual bool isTFTransformer(
    rviz_common::transformation::InternalFrameTransformerPtr transformer_internals)
  {
    auto tf_transformer =
      std::dynamic_pointer_cast<rviz_default_plugins::transformation::TFWrapper>(
      transformer_internals.lock());

    if (tf_transformer) {
      return true;
    }
    return false;
  }

  virtual void updatePropertiesVisibility()
  {
    if (using_tf_transformer_) {
      showDefaultProperties();
    } else {
      hideAllProperties();
    }
  }

  virtual void showDefaultProperties()
  {
    int number_of_properties = this->numChildren();
    for (int i = 0; i < number_of_properties; i++) {
      this->childAt(i)->show();
    }
  }

  virtual void hideAllProperties()
  {
    int number_of_properties = this->numChildren();
    for (int i = 0; i < number_of_properties; i++) {
      if (this->childAt(i)->objectName().contains("Status")) {
        continue;
      }
      this->childAt(i)->hide();
    }
  }

  bool using_tf_transformer_;
  std::string display_name_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__ONLY_TF_COMPATIBLE_BASE_DISPLAY_HPP_
