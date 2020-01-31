/*
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
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

#include <gtest/gtest.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <string>

#include "ament_index_cpp/get_resource.hpp"

TEST(CMake_Macro__Test, correctly_registers_plugin_in_ament_index) {
  std::string content;
  std::string prefix_path;
  ASSERT_TRUE(
    ament_index_cpp::get_resource(
      "rviz_ogre_media_exports", "rviz_rendering_tests",
      content,
      &prefix_path));
}

TEST(CMake_Macro__Test, ament_index_resource_file_has_correct_content) {
  std::string content;
  std::string prefix_path;
  ament_index_cpp::get_resource(
    "rviz_ogre_media_exports", "rviz_rendering_tests", content, &prefix_path);

  ASSERT_EQ(
    content,
    "rviz_rendering_tests/ogre_media_resources/scripts\n"
    "rviz_rendering_tests/ogre_media_resources/meshes\n");
}

TEST(CMake_Macro__Test, folders_are_installed_to_correct_location) {
  std::string content;
  std::string prefix_path;
  ament_index_cpp::get_resource(
    "rviz_ogre_media_exports", "rviz_rendering_tests", content, &prefix_path);

  struct stat info;
  std::string scripts = prefix_path + "/share/rviz_rendering_tests/ogre_media_resources/scripts";
  std::string meshes = prefix_path + "/share/rviz_rendering_tests/ogre_media_resources/meshes";
  ASSERT_EQ(stat(scripts.c_str(), &info), 0);
  ASSERT_EQ(stat(meshes.c_str(), &info), 0);
}
