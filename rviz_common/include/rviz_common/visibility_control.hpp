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

#ifndef RVIZ_COMMON__VISIBILITY_CONTROL_HPP_
#define RVIZ_COMMON__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RVIZ_COMMON_EXPORT __attribute__ ((dllexport))
    #define RVIZ_COMMON_IMPORT __attribute__ ((dllimport))
  #else
    #define RVIZ_COMMON_EXPORT __declspec(dllexport)
    #define RVIZ_COMMON_IMPORT __declspec(dllimport)
  #endif
  #ifdef RVIZ_COMMON_BUILDING_LIBRARY
    #define RVIZ_COMMON_PUBLIC RVIZ_COMMON_EXPORT
  #else
    #define RVIZ_COMMON_PUBLIC RVIZ_COMMON_IMPORT
  #endif
  #define RVIZ_COMMON_PUBLIC_TYPE RVIZ_COMMON_PUBLIC
  #define RVIZ_COMMON_LOCAL
#else
  #define RVIZ_COMMON_EXPORT __attribute__ ((visibility("default")))
  #define RVIZ_COMMON_IMPORT
  #if __GNUC__ >= 4
    #define RVIZ_COMMON_PUBLIC __attribute__ ((visibility("default")))
    #define RVIZ_COMMON_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RVIZ_COMMON_PUBLIC
    #define RVIZ_COMMON_LOCAL
  #endif
  #define RVIZ_COMMON_PUBLIC_TYPE
#endif

#endif  // RVIZ_COMMON__VISIBILITY_CONTROL_HPP_
