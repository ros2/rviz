/*
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#include <string>
#include <unordered_set>

#include "point_cloud_transport/point_cloud_transport.hpp"
#include "rcutils/logging_macros.h"
#include "rviz_default_plugins/displays/pointcloud/get_transport_from_topic.hpp"

namespace rviz_default_plugins
{
namespace displays
{

namespace
{
const std::unordered_set<std::string> & getKnownNonRawTransports()
{
  static const std::unordered_set<std::string> known_transports = []() {
      std::unordered_set<std::string> result;
      try {
        point_cloud_transport::PointCloudTransportLoader transport_loader;
        for (const std::string & declared_transport : transport_loader.getDeclaredTransports()) {
          const std::string transport =
            declared_transport.substr(declared_transport.find_last_of('/') + 1);
          if (transport != "raw") {
            result.insert(transport);
          }
        }
      } catch (const std::exception & e) {
        RCUTILS_LOG_WARN_NAMED(
          "rviz_default_plugins",
          "Failed to discover point_cloud_transport plugins: %s", e.what());
      } catch (...) {
        RCUTILS_LOG_WARN_NAMED(
          "rviz_default_plugins",
          "Failed to discover point_cloud_transport plugins: unknown error");
      }
      return result;
    }();
  return known_transports;
}
}  // namespace

bool isPointCloud2RawTransport(const std::string & topic)
{
  const std::string last_subtopic = topic.substr(topic.find_last_of('/') + 1);
  const auto & transports = getKnownNonRawTransports();
  return transports.find(last_subtopic) == transports.end();
}

std::string getPointCloud2TransportFromTopic(const std::string & topic)
{
  if (isPointCloud2RawTransport(topic)) {
    return "raw";
  }
  return topic.substr(topic.find_last_of('/') + 1);
}

std::string getPointCloud2BaseTopicFromTopic(const std::string & topic)
{
  if (isPointCloud2RawTransport(topic)) {
    return topic;
  }
  return topic.substr(0, topic.find_last_of('/'));
}

}  //  end namespace displays
}  //  end namespace rviz_default_plugins
