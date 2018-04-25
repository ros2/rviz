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
#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "../src/rviz_common/ros_integration/ros_node_storage_iface.hpp"
#include "../src/rviz_common/ros_integration/ros_node_storage.hpp"

using namespace ::testing;  // NOLINT

class MockRosNodeStorage : public rviz_common::ros_integration::RosNodeStorageIface
{
public:
  MOCK_METHOD2(store_rclcpp_node_by_name,
    void(const std::string & node_name, std::shared_ptr<rclcpp::Node> node));
  MOCK_METHOD1(get_rclcpp_node_by_name,
    rclcpp::Node::SharedPtr(const std::string & node_name));
  MOCK_METHOD1(has_rclcpp_node_by_name, bool(const std::string & node_name));
  MOCK_METHOD0(clear_rclcpp_nodes, void());
};

class RosNodeAbstractionTestFixture : public Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  RosNodeAbstractionTestFixture()
  {
    ros_node_storage_ = std::make_unique<MockRosNodeStorage>();
  }

  std::unique_ptr<rviz_common::ros_integration::RosNodeStorageIface> ros_node_storage_;
  std::string node_name_ = "node_name";
};

TEST_F(RosNodeAbstractionTestFixture,
  RosNodeAbstraction_creates_a_node_with_given_name_in_storage) {
  EXPECT_CALL(
    *dynamic_cast<MockRosNodeStorage *>(ros_node_storage_.get()),
    has_rclcpp_node_by_name(node_name_))
  .WillOnce(Return(false));
  EXPECT_CALL(
    *dynamic_cast<MockRosNodeStorage *>(ros_node_storage_.get()),
    store_rclcpp_node_by_name(node_name_, AllOf(
      A<rclcpp::Node::SharedPtr>(),
      Pointee(Property(&rclcpp::Node::get_name, StrEq(node_name_)))
    ))
  )
  .Times(1);

  auto node = rviz_common::ros_integration::RosNodeAbstraction(
    node_name_, std::move(ros_node_storage_));

  ASSERT_EQ(node.get_node_name(), node_name_);
}

TEST_F(RosNodeAbstractionTestFixture,
  RosNodeAbstraction_reuses_existing_node_with_same_name) {
  EXPECT_CALL(
    *dynamic_cast<MockRosNodeStorage *>(ros_node_storage_.get()),
    has_rclcpp_node_by_name(node_name_))
  .WillOnce(Return(true));
  EXPECT_CALL(
    *dynamic_cast<MockRosNodeStorage *>(ros_node_storage_.get()), store_rclcpp_node_by_name(_, _))
  .Times(0);

  auto node = rviz_common::ros_integration::RosNodeAbstraction(
    node_name_, std::move(ros_node_storage_));

  ASSERT_EQ(node.get_node_name(), node_name_);
}
