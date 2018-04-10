/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#ifndef RVIZ_COMMON__YAML_CONFIG_WRITER_HPP_
#define RVIZ_COMMON__YAML_CONFIG_WRITER_HPP_

#include <ostream>

#include "rviz_common/config.hpp"
#include "rviz_common/visibility_control.hpp"

namespace YAML
{
class Emitter;
}

namespace rviz_common
{

class RVIZ_COMMON_PUBLIC YamlConfigWriter
{
public:
  /// Constructor.
  /**
   * Writer starts in a non-error state.
   */
  YamlConfigWriter();

  /// Write config data to a file.
  /**
   * This potentially changes the return values of error() and statusMessage().
   */
  void writeFile(const Config & config, const QString & filename);

  /// Write config data to a string, and return it.
  /**
   * This potentially changes the return values of error() and statusMessage().
   */
  QString writeString(const Config & config, const QString & filename = "data string");

  /// Write config data to a std::ostream.
  /**
   * This potentially changes the return values of error() and statusMessage().
   */
  void writeStream(
    const Config & config,
    std::ostream & out,
    const QString & filename = "data stream");

  /// Return true if the latest write operation had an error.
  bool error();

  /// Return an error message if the latest write call had an error, else the empty string.
  QString errorMessage();

private:
  void writeConfigNode(const Config & config, YAML::Emitter & emitter);

  QString message_;
  bool error_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__YAML_CONFIG_WRITER_HPP_
