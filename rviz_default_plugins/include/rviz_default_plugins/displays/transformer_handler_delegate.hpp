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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__TRANSFORMER_HANDLER_DELEGATE_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__TRANSFORMER_HANDLER_DELEGATE_HPP_

#include <string>
#include <memory>

#include <QString>  // NOLINT
#include <QObject>  //NOLINT

#include "rviz_common/display_context.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"
#include "rviz_common/transformation/transformation_manager.hpp"
#include "rviz_default_plugins/transformation/tf_wrapper.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace displays
{

/** @brief Helper base class  for TransformerHandlerDelegate, needed because Qt's moc and c++
 * templates don't work nicely together. Not intended to be used directly.
 */
class _TransformerHandlerDelegate : public QObject
{
  Q_OBJECT

public:
  _TransformerHandlerDelegate(
    rviz_common::Display * display_,
    const std::string & display_name,
    const std::string & transformer_name)
    : display_(display_),
    display_name_(display_name),
    allowed_transformer_name_(transformer_name),
    using_allowed_transformer_(true),
    disabled_by_user_(false)
  {}

  void initialize(rviz_common::DisplayContext * context)
  {
    context_ = context;
    connect(
      context_->getTransformationManager(),
      SIGNAL(transformerChanged(std::shared_ptr<rviz_common::transformation::FrameTransformer>)),
      this,
      SLOT(transformerChanged(std::shared_ptr<rviz_common::transformation::FrameTransformer>)));
//    connect(display_, SIGNAL(changed()), this, SLOT(enabledChanged()));
  }

protected Q_SLOTS:
  virtual void transformerChanged(
    std::shared_ptr<rviz_common::transformation::FrameTransformer> new_transformer) = 0;
//  virtual void enabledChanged() = 0;

protected:
  rviz_common::Display * display_;
  std::string display_name_;
  std::string allowed_transformer_name_;
  bool using_allowed_transformer_;
  bool disabled_by_user_;
  rviz_common::DisplayContext * context_;
};

/** \brief Convenience helper class for displays that can only work with TFTransformer.
 *  It helps handling the change of the transformer.
 */
template<typename AllowedTransformerType>
class RVIZ_DEFAULT_PLUGINS_PUBLIC TransformerHandlerDelegate : public _TransformerHandlerDelegate
{
public:
  TransformerHandlerDelegate(
    rviz_common::Display * display_,
    const std::string & display_name,
    const std::string & transformer_name)
  : _TransformerHandlerDelegate(display_, display_name, transformer_name)
  {}

  ~TransformerHandlerDelegate() override = default;

  void updateDisplayAccordingToTransformerType(
    rviz_common::transformation::InternalFrameTransformerPtr transformer_internal)
  {
    using_allowed_transformer_ = isAllowedTransformer(transformer_internal);

    if (!using_allowed_transformer_) {
      disableDisplayAndSetErrorStatus();
    } else {
      enableDisplayAndDeleteErrorStatus();
    }
  }

private:
  void transformerChanged(
    std::shared_ptr<rviz_common::transformation::FrameTransformer> new_transformer) override
  {
    if (using_allowed_transformer_ != isAllowedTransformer(new_transformer->getInternals())) {
      updateDisplayAccordingToTransformerType(context_->getFrameManager()->getInternalPtr());
    }
  }

  virtual bool isAllowedTransformer(
    rviz_common::transformation::InternalFrameTransformerPtr transformer_internals)
  {
    auto transformer =
      std::dynamic_pointer_cast<AllowedTransformerType>(
      transformer_internals.lock());

    if (transformer) {
      return true;
    }
    return false;
  }

  void disableDisplayAndSetErrorStatus()
  {
    if (!display_->isEnabled()) {
      disabled_by_user_ = true;
    } else if (display_->isEnabled()){
      disabled_by_user_ = false;
    }

    display_->setEnabled(false);
    display_->setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Transformer",
      QString::fromStdString(
        display_name_ +
          " display works only with " + allowed_transformer_name_ + " Transformer"));
  }

  void enableDisplayAndDeleteErrorStatus()
  {
    display_->deleteStatusStd("Transformer");
    if (!disabled_by_user_) {
      display_->setEnabled(true);
    }
  }

//  void enabledChanged() override
//  {
//    if (enabled_changed_by_user_) {
//      disableDisplayAndSetErrorStatus(true);
//    }
//  }
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__TRANSFORMER_HANDLER_DELEGATE_HPP_
