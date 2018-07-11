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

/// Redirectable logging macros and functions.
/**
 * \file logging.hpp
 * This file contains logging macros which are used within this library.
 * The macros wrap some generic logging functions, which can be redirected
 * using custom logging handlers via the set_logging_handlers() function.
 * By default logging goes to stdout or stderr based on the severity.
 *
 * The install_rviz_rendering_log_handlers() function can be called after
 * setting the logging handlers with set_logging_handlers() to propogate these
 * handlers to the rviz_rendering libraries logging system as well.
 *
 * There are four logging levels: debug, info, warning, and error.
 * For each there is a simple macro which takes a single string, and a more
 * complex macro for taking stream arguments in the style of sstring.
 *
 * For example:
 *
 *   RVIZ_COMMON_LOG_INFO("hello world")
 *   RVIZ_COMMON_LOG_WARNING_STREAM("hello " << "world: " << 42)
 */

#ifndef RVIZ_COMMON__LOGGING_HPP_
#define RVIZ_COMMON__LOGGING_HPP_

#include <memory>
#include <sstream>
#include <string>

#include "rviz_rendering/logging_handler.hpp"
#include "rviz_common/visibility_control.hpp"

#define RVIZ_COMMON_LOG_DEBUG(msg) do { \
    rviz_common::log_debug(msg, __FILE__, __LINE__); \
} while (0)

#define RVIZ_COMMON_LOG_DEBUG_STREAM(args) do { \
    std::stringstream __ss; \
    __ss << args; \
    rviz_common::log_debug(__ss.str(), __FILE__, __LINE__); \
} while (0)

#define RVIZ_COMMON_LOG_INFO(msg) do { \
    rviz_common::log_info(msg, __FILE__, __LINE__); \
} while (0)

#define RVIZ_COMMON_LOG_INFO_STREAM(args) do { \
    std::stringstream __ss; \
    __ss << args; \
    rviz_common::log_info(__ss.str(), __FILE__, __LINE__); \
} while (0)

#define RVIZ_COMMON_LOG_WARNING(msg) do { \
    rviz_common::log_warning(msg, __FILE__, __LINE__); \
} while (0)

#define RVIZ_COMMON_LOG_WARNING_STREAM(args) do { \
    std::stringstream __ss; \
    __ss << args; \
    rviz_common::log_warning(__ss.str(), __FILE__, __LINE__); \
} while (0)

#define RVIZ_COMMON_LOG_ERROR(msg) do { \
    rviz_common::log_error(msg, __FILE__, __LINE__); \
} while (0)

#define RVIZ_COMMON_LOG_ERROR_STREAM(args) do { \
    std::stringstream __ss; \
    __ss << args; \
    rviz_common::log_error(__ss.str(), __FILE__, __LINE__); \
} while (0)

namespace rviz_common
{

using LoggingHandler = std::function<
  void (const std::string & message, const std::string & file_name, size_t line_number)
>;

/// Set the given logging handlers globally.
/**
 * All log traffic is routed through these logging functions.
 */
RVIZ_COMMON_PUBLIC
void
set_logging_handlers(
  rviz_common::LoggingHandler debug_handler,
  rviz_common::LoggingHandler info_handler,
  rviz_common::LoggingHandler warning_handler,
  rviz_common::LoggingHandler error_handler);

/// Install the current logging handlers into the rviz_rendering logging system.
RVIZ_COMMON_PUBLIC
void
install_rviz_rendering_log_handlers();

RVIZ_COMMON_PUBLIC
void
log_debug(const std::string & message, const std::string & file_name, size_t line_number);

RVIZ_COMMON_PUBLIC
void
log_info(const std::string & message, const std::string & file_name, size_t line_number);

RVIZ_COMMON_PUBLIC
void
log_warning(const std::string & message, const std::string & file_name, size_t line_number);

RVIZ_COMMON_PUBLIC
void
log_error(const std::string & message, const std::string & file_name, size_t line_number);

}  // namespace rviz_common

#endif  // RVIZ_COMMON__LOGGING_HPP_
