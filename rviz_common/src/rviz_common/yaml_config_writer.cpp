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

#include "rviz_common/yaml_config_writer.hpp"

#include <fstream>

#include "yaml-cpp/emitter.h"

namespace rviz_common
{

YamlConfigWriter::YamlConfigWriter()
: error_(false)
{}

void YamlConfigWriter::writeFile(const Config & config, const QString & filename)
{
  try {
    std::ofstream out(qPrintable(filename));
    if (out) {
      writeStream(config, out, filename);
    } else {
      error_ = true;
      message_ = "Failed to open " + filename + " for writing.";
    }
  } catch (const std::exception & ex) {
    error_ = true;
    message_ = ex.what();
  }
}

QString YamlConfigWriter::writeString(const Config & config, const QString & filename)
{
  std::stringstream out;
  writeStream(config, out, filename);
  if (!error_) {
    return QString::fromStdString(out.str() );
  } else {
    return "";
  }
}

void YamlConfigWriter::writeStream(
  const Config & config,
  std::ostream & out,
  const QString & filename)
{
  (void) filename;
  error_ = false;
  message_ = "";
  YAML::Emitter emitter;
  writeConfigNode(config, emitter);
  if (!error_) {
    out << emitter.c_str() << std::endl;
  }
}

bool YamlConfigWriter::error()
{
  return error_;
}

QString YamlConfigWriter::errorMessage()
{
  return message_;
}

void YamlConfigWriter::writeConfigNode(const Config & config, YAML::Emitter & emitter)
{
  switch (config.getType() ) {
    case Config::List:
      {
        emitter << YAML::BeginSeq;
        for (int i = 0; i < config.listLength(); i++) {
          writeConfigNode(config.listChildAt(i), emitter);
        }
        emitter << YAML::EndSeq;
        break;
      }
    case Config::Map:
      {
        emitter << YAML::BeginMap;
        Config::MapIterator map_iter = config.mapIterator();
        while (map_iter.isValid() ) {
          Config child = map_iter.currentChild();

          emitter << YAML::Key;
          emitter << map_iter.currentKey().toStdString();
          emitter << YAML::Value;
          writeConfigNode(child, emitter);

          map_iter.advance();
        }
        emitter << YAML::EndMap;
        break;
      }
    case Config::Value:
      {
        QString value = config.getValue().toString();
        if (value.size() == 0) {
          emitter << YAML::DoubleQuoted << "";
        } else {
          emitter << value.toStdString();
        }
        break;
      }
    default:
      emitter << YAML::Null;
      break;
  }
}

}  // namespace rviz_common
