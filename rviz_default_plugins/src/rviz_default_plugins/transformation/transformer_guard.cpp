// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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

#include "rviz_default_plugins/transformation/transformer_guard.hpp"

#include <string>

#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/transformation/transformation_manager.hpp"

namespace rviz_default_plugins
{
namespace transformation
{

_TransformerGuard::_TransformerGuard(
  rviz_common::Display * display,
  const std::string & transformer_name)
: display_(display),
  allowed_transformer_name_(transformer_name),
  using_allowed_transformer_(true),
  display_disabled_by_user_(false),
  context_(nullptr)
{}

void
_TransformerGuard::initialize(rviz_common::DisplayContext * context)
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

}  // namespace transformation
}  // namespace rviz_default_plugins
