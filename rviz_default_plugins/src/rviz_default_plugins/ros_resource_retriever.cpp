// Copyright 2025 Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "rviz_default_plugins/ros_resource_retriever.hpp"

#include <cinttypes>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <resource_retriever/memory_resource.hpp>
#include <resource_retriever/plugins/retriever_plugin.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_resource_interfaces/srv/get_resource.hpp>

rclcpp::Node::SharedPtr
get_ros_node_from(
  const rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr & weak_ros_iface)
{
  auto ros_iface = weak_ros_iface.lock();
  if (!ros_iface) {
    throw std::invalid_argument("ROS node abstraction interface not valid");
  }
  return ros_iface->get_raw_node();
}

RosResourceRetriever::RosResourceRetriever(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr weak_ros_iface)
: ros_node_(get_ros_node_from(weak_ros_iface)),
  logger_(ros_node_->get_logger().get_child("ros_resource_retriever"))
{
  this->logger_ = ros_node_->get_logger().get_child("ros_resource_retriever");

  // Create a client with a custom callback group that will not be included in the main executor.
  callback_group_ = ros_node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  this->client_ = ros_node_->create_client<GetResource>(
    service_name.data(),
    rclcpp::ServicesQoS(),
    callback_group_);

  // Add the callback group to the executor so we can spin on it later.
  executor_.add_callback_group(callback_group_, ros_node_->get_node_base_interface());
}

std::string RosResourceRetriever::name()
{
  return "rviz_default_plugins::RosResourceRetriever";
}

bool RosResourceRetriever::can_handle(const std::string & url)
{
  return !url.empty();
}

resource_retriever::ResourceSharedPtr
RosResourceRetriever::get_shared(const std::string & url)
{
  RCLCPP_DEBUG(this->logger_, "Getting resource: %s", url.c_str());

  // First check for a cache hit.
  std::string etag;
  auto it = cached_resources_.find(url);
  if (it != cached_resources_.end()) {
    etag = it->second.first;
    // If the etag was not set, then the server doesn't do caching, just return what we have.
    if (etag.empty()) {
      RCLCPP_DEBUG(this->logger_, "Resource '%s' cached without etag, returning.", url.c_str());
      return it->second.second;
    }
  }

  // Request the resource with an etag, if it is set.
  RCLCPP_DEBUG(
    this->logger_,
    "Requesting resource '%s'%s.",
    url.c_str(),
    etag.empty() ? "" : (" with etag '" + etag + "'").c_str());
  auto req = std::make_shared<GetResource::Request>();
  req->path = url;
  req->etag = etag;
  auto result = this->client_->async_send_request(req);

  using namespace std::chrono_literals;
  auto maximum_wait_time = 3s;

  if (executor_.spin_until_future_complete(result, maximum_wait_time) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->logger_, "Timeout: Not able to call the service %s", service_name.data());
    this->client_->remove_pending_request(result);
    return nullptr;
  }

  auto res = result.get();
  std::shared_ptr<resource_retriever::Resource> memory_resource = nullptr;
  switch (res->status_code) {
    case rviz_resource_interfaces::srv::GetResource::Response::OK:
      RCLCPP_DEBUG(
        this->logger_,
        "Received resource '%s' with etag '%s', caching and returning %zu bytes.",
        res->expanded_path.c_str(),
        res->etag.c_str(),
        res->body.size());
      memory_resource =
        std::make_shared<resource_retriever::Resource>(url, res->expanded_path, res->body);
      cached_resources_.insert({url, {res->etag, memory_resource}});
      return memory_resource;
    case rviz_resource_interfaces::srv::GetResource::Response::NOT_MODIFIED:
      RCLCPP_DEBUG(
        this->logger_,
        "Resource '%s' with etag '%s' was not modified, returning cached value.",
        res->expanded_path.c_str(),
        res->etag.c_str());
      if (etag != res->etag) {
        RCLCPP_WARN(
          this->logger_,
          "Unexpectedly got a different etag values ('%s' vs '%s') for resource '%s' "
          "with a NOT_MODIFIED status_code. This will not stop the resource "
          "from loading, but indicates some issue with the caching logic.",
          res->expanded_path.c_str(),
          etag.c_str(),
          res->etag.c_str());
      }
      return it->second.second;
      break;
    case rviz_resource_interfaces::srv::GetResource::Response::ERROR:
      RCLCPP_DEBUG(
        this->logger_,
        "Received an unexpected error when getting resource '%s': %s",
        url.c_str(),
        res->error_reason.c_str());
      return nullptr;
      break;
    default:
      RCLCPP_ERROR(
        this->logger_,
        "Unexpected status_code from resource ROS Service '%s' for resource '%s': %" PRId32,
        service_name.data(),
        url.c_str(),
        res->status_code);
      return nullptr;
      break;
  }
}
