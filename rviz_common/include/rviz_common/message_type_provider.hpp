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

#ifndef RVIZ_COMMON__MESSAGE_TYPE_PROVIDER_HPP_
#define RVIZ_COMMON__MESSAGE_TYPE_PROVIDER_HPP_

#include <QMap>  // NOLINT: cpplint cannot handle include order here
#include <QSet>  // NOLINT: cpplint cannot handle include order here
#include <QString>  // NOLINT: cpplint cannot handle include order here

namespace rviz_common
{

/// Pluginlib interface to register message types for display classes.
/**
 * A display's supported message types are normally declared statically with
 * <message_type> tags in its plugin description xml.  Displays whose
 * supported types are only known at runtime (e.g. the Image display, which
 * discovers them from the installed image_transport plugins) can export a
 * MessageTypeProvider plugin instead.  The DisplayFactory loads all declared
 * providers before the message types are first queried, so that dialogs like
 * "Add display by topic" know about these types before any instance of the
 * display exists.
 *
 * Export implementations with
 * PLUGINLIB_EXPORT_CLASS(..., rviz_common::MessageTypeProvider) and declare
 * them with base_class_type="rviz_common::MessageTypeProvider" in the plugin
 * description xml.
 */
class MessageTypeProvider
{
public:
  virtual ~MessageTypeProvider() = default;

  /// Return the supported message types per display class id.
  /**
   * The map key is the display class id (e.g. "rviz_default_plugins/Image"),
   * the value is the set of fully qualified message types
   * (e.g. "sensor_msgs/msg/Image") to register for it.  The returned types
   * are merged with the statically declared ones.
   */
  virtual QMap<QString, QSet<QString>> getMessageTypes() = 0;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__MESSAGE_TYPE_PROVIDER_HPP_
