/*
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#ifndef RVIZ_DEFAULT_PLUGINS__TRANSFORMATION__TRANSFORMER_GUARD_HPP_
#define RVIZ_DEFAULT_PLUGINS__TRANSFORMATION__TRANSFORMER_GUARD_HPP_

#include <string>
#include <memory>

#include <QString>  // NOLINT
#include <QObject>  //NOLINT

#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"
#include "rviz_common/transformation/transformation_manager.hpp"
#include "rviz_default_plugins/transformation/tf_wrapper.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace transformation
{

/// Helper class for TransformerGuard.
/**
 * This is needed because Qt's moc and c++ templates do not work nicely
 * together (i.e. a Q_OBJECT may not be a template class).
 * Not intended to be used directly, use TransformerGuard instead.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC _TransformerGuard : public QObject
{
  Q_OBJECT

public:
  _TransformerGuard(
    rviz_common::Display * display,
    const std::string & transformer_name)
  : display_(display),
    allowed_transformer_name_(transformer_name),
    using_allowed_transformer_(true),
    display_disabled_by_user_(false),
    context_(nullptr)
  {}

  void
  initialize(rviz_common::DisplayContext * context)
  {
    context_ = context;
    connect(
      context_->getTransformationManager(),
      SIGNAL(transformerChanged(std::shared_ptr<rviz_common::transformation::FrameTransformer>)),
      this,
      SLOT(transformerChanged(std::shared_ptr<rviz_common::transformation::FrameTransformer>)));
    connect(display_, SIGNAL(changed()), this, SLOT(displayEnabledChanged()));

    if (!checkTransformer()) {
      using_allowed_transformer_ = false;
      Q_EMIT (display_->changed());
    }
  }

  virtual
  bool
  checkTransformer() = 0;

protected Q_SLOTS:
  virtual
  void
  transformerChanged(
    std::shared_ptr<rviz_common::transformation::FrameTransformer> new_transformer) = 0;
  virtual
  void
  displayEnabledChanged() = 0;

protected:
  rviz_common::Display * display_;
  std::string allowed_transformer_name_;
  bool using_allowed_transformer_;
  bool display_disabled_by_user_;
  rviz_common::DisplayContext * context_;
};

/// Convenience class for displays that can only work with a specific transformer.
/**
 * For example, some displays may only work with the tf2 backed transformer.
 * It helps handling the change of the transformer.
 */
template<typename AllowedTransformerType>
class TransformerGuard : public _TransformerGuard
{
public:
  // The transformer_name is only needed to set the error status.
  TransformerGuard(
    rviz_common::Display * display,
    const std::string & transformer_name)
  : _TransformerGuard(display, transformer_name)
  {}

  ~TransformerGuard() override = default;

  bool
  checkTransformer() override
  {
    return isAllowedTransformer(context_->getFrameManager()->getTransformer());
  }

  void
  updateDisplayAccordingToTransformerType(
    std::shared_ptr<rviz_common::transformation::FrameTransformer> transformer)
  {
    using_allowed_transformer_ = isAllowedTransformer(transformer);

    if (!using_allowed_transformer_) {
      display_disabled_by_user_ = !display_->isEnabled();
      disableDisplayAndSetErrorStatus();
    } else {
      enableDisplayAndDeleteErrorStatus();
    }
  }

private:
  void
  transformerChanged(
    std::shared_ptr<rviz_common::transformation::FrameTransformer> new_transformer) override
  {
    if (using_allowed_transformer_ != isAllowedTransformer(new_transformer)) {
      updateDisplayAccordingToTransformerType(context_->getFrameManager()->getTransformer());
    }
  }

  void
  displayEnabledChanged() override
  {
    if (!using_allowed_transformer_) {
      disableDisplayAndSetErrorStatus();
    }
  }

  virtual
  bool
  isAllowedTransformer(
    std::shared_ptr<rviz_common::transformation::FrameTransformer> transformer)
  {
    auto derived_transformer = std::dynamic_pointer_cast<AllowedTransformerType>(transformer);
    return static_cast<bool>(derived_transformer);
  }

  void
  setErrorStatus()
  {
    display_->setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Transformer",
      QString::fromStdString(
        "The display works only with " + allowed_transformer_name_ + " Transformer"));
  }

  void
  enableDisplayAndDeleteErrorStatus()
  {
    display_->deleteStatusStd("Transformer");
    if (!display_disabled_by_user_) {
      display_->setEnabled(true);
    }
  }

  void
  disableDisplayAndSetErrorStatus()
  {
    display_->setEnabled(false);
    setErrorStatus();
  }
};

}  // namespace transformation
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__TRANSFORMATION__TRANSFORMER_GUARD_HPP_
