/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include "retriever.h"

#include <curl/curl.h>

#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace resource_retriever
{

class CURLStaticInit
{
public:
  CURLStaticInit()
  : initialized_(false)
  {
    CURLcode ret = curl_global_init(CURL_GLOBAL_ALL);
    if (ret != 0) {
      fprintf(stderr, "Error initializing libcurl! retcode = %d", ret);
    } else {
      initialized_ = true;
    }
  }

  ~CURLStaticInit()
  {
    if (initialized_) {
      curl_global_cleanup();
    }
  }

  bool initialized_;
};
static CURLStaticInit g_curl_init;

Retriever::Retriever()
{
  curl_handle_ = curl_easy_init();
}

Retriever::~Retriever()
{
  if (curl_handle_) {
    curl_easy_cleanup(curl_handle_);
  }
}

struct MemoryBuffer
{
  std::vector<uint8_t> v;
};

size_t curlWriteFunc(void * buffer, size_t size, size_t nmemb, void * userp)
{
  MemoryBuffer * membuf = (MemoryBuffer *)userp;

  size_t prev_size = membuf->v.size();
  membuf->v.resize(prev_size + size * nmemb);
  memcpy(&membuf->v[prev_size], buffer, size * nmemb);

  return size * nmemb;
}

MemoryResource Retriever::get(const std::string & url)
{
  std::string mod_url = url;
  if (url.find("package://") == 0) {
    mod_url.erase(0, strlen("package://"));
    size_t pos = mod_url.find("/");
    if (pos == std::string::npos) {
      throw Exception(url, "Could not parse package:// format into file:// format");
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    // std::string package_path = ros::package::getPath(package);
    std::string package_path = "/Users/william/hsr_ws/src/hsr_meshes";

    if (package_path.empty()) {
      throw Exception(url, "Package [" + package + "] does not exist");
    }

    mod_url = "file://" + package_path + mod_url;
  }

  curl_easy_setopt(curl_handle_, CURLOPT_URL, mod_url.c_str());
  curl_easy_setopt(curl_handle_, CURLOPT_WRITEFUNCTION, curlWriteFunc);

  char error_buffer[CURL_ERROR_SIZE];
  curl_easy_setopt(curl_handle_, CURLOPT_ERRORBUFFER, error_buffer);

  MemoryResource res;
  MemoryBuffer buf;
  curl_easy_setopt(curl_handle_, CURLOPT_WRITEDATA, &buf);

  CURLcode ret = curl_easy_perform(curl_handle_);
  if (ret != 0) {
    throw Exception(mod_url, error_buffer);
  } else if (!buf.v.empty()) {
    res.size = buf.v.size();
    res.data.reset(new uint8_t[res.size], std::default_delete<uint8_t[]>());
    memcpy(res.data.get(), &buf.v[0], res.size);
  }

  return res;
}

}  // namespace resource_retriever
