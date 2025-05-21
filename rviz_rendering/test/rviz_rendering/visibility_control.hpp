// Copyright 2023 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEST_RVIZ_RENDERING__VISIBILITY_CONTROL_HPP_  // NOLINT
#define TEST_RVIZ_RENDERING__VISIBILITY_CONTROL_HPP_  // NOLINT

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TEST_RVIZ_RENDERING_EXPORT __attribute__ ((dllexport))
    #define TEST_RVIZ_RENDERING_IMPORT __attribute__ ((dllimport))
  #else
    #define TEST_RVIZ_RENDERING_EXPORT __declspec(dllexport)
    #define TEST_RVIZ_RENDERING_IMPORT __declspec(dllimport)
  #endif
  #ifdef TEST_RVIZ_RENDERING_BUILDING_LIBRARY
    #define TEST_RVIZ_RENDERING_PUBLIC TEST_RVIZ_RENDERING_EXPORT
  #else
    #define TEST_RVIZ_RENDERING_PUBLIC TEST_RVIZ_RENDERING_IMPORT
  #endif
  #define TEST_RVIZ_RENDERING_PUBLIC_TYPE TEST_RVIZ_RENDERING_PUBLIC
  #define TEST_RVIZ_RENDERING_LOCAL
#else
  #define TEST_RVIZ_RENDERING_EXPORT __attribute__ ((visibility("default")))
  #define TEST_RVIZ_RENDERING_IMPORT
  #if __GNUC__ >= 4
    #define TEST_RVIZ_RENDERING_PUBLIC __attribute__ ((visibility("default")))
    #define TEST_RVIZ_RENDERING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TEST_RVIZ_RENDERING_PUBLIC
    #define TEST_RVIZ_RENDERING_LOCAL
  #endif
  #define TEST_RVIZ_RENDERING_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // TEST_RVIZ_RENDERING__VISIBILITY_CONTROL_HPP_  // NOLINT
