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

#ifndef RVIZ_COMMON__YAML_CONFIG_READER_HPP_
#define RVIZ_COMMON__YAML_CONFIG_READER_HPP_

#include <istream>

#include "rviz_common/config.hpp"
#include "rviz_common/visibility_control.hpp"

#include "yaml-cpp/yaml.h"

namespace rviz_common
{

class RVIZ_COMMON_PUBLIC YamlConfigReader
{
public:
  /// Constructor.
  /**
   * Object begins in a no-error state.
   */
  YamlConfigReader();

  /// Read config data from a file.
  /**
   * This potentially changes the return value sof error(), statusMessage(),
   * and config().
   */
  void readFile(Config & config, const QString & filename);

  /// Read config data from a string.
  /**
   * This potentially changes the return value sof error(), statusMessage(),
   * and config().
   */
  void readString(Config & config, const QString & data, const QString & filename = "data string");

  /// Read config data from a std::istream.
  /**
   * This potentially changes the return value sof error(), statusMessage(),
   * and config().
   */
  void readStream(Config & config, std::istream & in, const QString & filename = "data stream");

  /// Return true if the latest readFile() or readString() call had an error.
  bool error();

  /// Return an error message if the latest read call had an error, or the empty string if not.
  QString errorMessage();

private:
  void readYamlNode(Config & config, const YAML::Node & yaml_node);

  QString message_;
  bool error_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__YAML_CONFIG_READER_HPP_
