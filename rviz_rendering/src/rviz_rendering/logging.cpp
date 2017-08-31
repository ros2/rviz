/*
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

#include "rviz_rendering/logging.hpp"

#include <cstdio>
#include <memory>
#include <mutex>
#include <string>

#include "rviz_rendering/logging_handler.hpp"

namespace
{

static rviz_rendering::LoggingHandler __debug_logging_handler = [](
  const std::string & message,
  const std::string & file_name,
  size_t line_number)
  {
    printf(
      "[rviz_rendering:debug] %s, at %s:%zu\n", message.c_str(), file_name.c_str(), line_number);
  };
static rviz_rendering::LoggingHandler __info_logging_handler = [](
  const std::string & message,
  const std::string & file_name,
  size_t line_number)
  {
    printf(
      "[rviz_rendering:info] %s, at %s:%zu\n", message.c_str(), file_name.c_str(), line_number);
  };
static rviz_rendering::LoggingHandler __warning_logging_handler = [](
  const std::string & message,
  const std::string & file_name,
  size_t line_number)
  {
    fprintf(
      stderr,
      "[rviz_rendering:warning] %s, at %s:%zu\n", message.c_str(), file_name.c_str(), line_number);
  };
static rviz_rendering::LoggingHandler __error_logging_handler = [](
  const std::string & message,
  const std::string & file_name,
  size_t line_number)
  {
    fprintf(
      stderr,
      "[rviz_rendering:error] %s, at %s:%zu\n", message.c_str(), file_name.c_str(), line_number);
  };

static std::mutex __logging_mutex;

}  // namespace

namespace rviz_rendering
{

void
set_logging_handlers(
  rviz_rendering::LoggingHandler debug_handler,
  rviz_rendering::LoggingHandler info_handler,
  rviz_rendering::LoggingHandler warning_handler,
  rviz_rendering::LoggingHandler error_handler)
{
  std::lock_guard<std::mutex> logging_lock(__logging_mutex);
  __debug_logging_handler = debug_handler;
  __info_logging_handler = info_handler;
  __warning_logging_handler = warning_handler;
  __error_logging_handler = error_handler;
}

void
log_debug(const std::string & message, const std::string & file_name, size_t line_number)
{
  std::lock_guard<std::mutex> logging_lock(__logging_mutex);
  __debug_logging_handler(message, file_name, line_number);
}

void
log_info(const std::string & message, const std::string & file_name, size_t line_number)
{
  std::lock_guard<std::mutex> logging_lock(__logging_mutex);
  __info_logging_handler(message, file_name, line_number);
}

void
log_warning(const std::string & message, const std::string & file_name, size_t line_number)
{
  std::lock_guard<std::mutex> logging_lock(__logging_mutex);
  __warning_logging_handler(message, file_name, line_number);
}

void
log_error(const std::string & message, const std::string & file_name, size_t line_number)
{
  std::lock_guard<std::mutex> logging_lock(__logging_mutex);
  __error_logging_handler(message, file_name, line_number);
}

}  // namespace rviz_rendering
