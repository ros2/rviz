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

#ifndef RVIZ_RENDERING__LOGGING_HPP_
#define RVIZ_RENDERING__LOGGING_HPP_

#include <memory>
#include <sstream>
#include <string>

#include "rviz_rendering/logging_handler.hpp"
#include "rviz_rendering/visibility_control.hpp"

#define RVIZ_RENDERING_LOG_DEBUG(msg) do { \
    rviz_rendering::log_debug(msg, __FILE__, __LINE__); \
} while (0)

#define RVIZ_RENDERING_LOG_DEBUG_STREAM(args) do { \
    std::stringstream __ss; \
    __ss << args; \
    rviz_rendering::log_debug(__ss.str(), __FILE__, __LINE__); \
} while (0)

#define RVIZ_RENDERING_LOG_INFO(msg) do { \
    rviz_rendering::log_info(msg, __FILE__, __LINE__); \
} while (0)

#define RVIZ_RENDERING_LOG_INFO_STREAM(args) do { \
    std::stringstream __ss; \
    __ss << args; \
    rviz_rendering::log_info(__ss.str(), __FILE__, __LINE__); \
} while (0)

#define RVIZ_RENDERING_LOG_WARNING(msg) do { \
    rviz_rendering::log_warning(msg, __FILE__, __LINE__); \
} while (0)

#define RVIZ_RENDERING_LOG_WARNING_STREAM(args) do { \
    std::stringstream __ss; \
    __ss << args; \
    rviz_rendering::log_warning(__ss.str(), __FILE__, __LINE__); \
} while (0)

#define RVIZ_RENDERING_LOG_ERROR(msg) do { \
    rviz_rendering::log_error(msg, __FILE__, __LINE__); \
} while (0)

#define RVIZ_RENDERING_LOG_ERROR_STREAM(args) do { \
    std::stringstream __ss; \
    __ss << args; \
    rviz_rendering::log_error(__ss.str(), __FILE__, __LINE__); \
} while (0)

namespace rviz_rendering
{

RVIZ_RENDERING_PUBLIC
void
set_logging_handlers(
  rviz_rendering::LoggingHandler debug_handler,
  rviz_rendering::LoggingHandler info_handler,
  rviz_rendering::LoggingHandler warning_handler,
  rviz_rendering::LoggingHandler error_handler);

RVIZ_RENDERING_PUBLIC
void
log_debug(const std::string & message, const std::string & file_name, size_t line_number);

RVIZ_RENDERING_PUBLIC
void
log_info(const std::string & message, const std::string & file_name, size_t line_number);

RVIZ_RENDERING_PUBLIC
void
log_warning(const std::string & message, const std::string & file_name, size_t line_number);

RVIZ_RENDERING_PUBLIC
void
log_error(const std::string & message, const std::string & file_name, size_t line_number);

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__LOGGING_HPP_
