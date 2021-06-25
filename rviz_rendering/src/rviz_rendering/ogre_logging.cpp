/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz_rendering/ogre_logging.hpp"

#include <memory>
#include <string>

#include <OgreLogManager.h>
#include <OgreLog.h>

#include "rviz_rendering/logging.hpp"

class CustomOgreLogListener : public Ogre::LogListener
{
public:
  CustomOgreLogListener()
  : min_lml(Ogre::LML_CRITICAL) {}

  virtual ~CustomOgreLogListener() {}

  virtual
  void
  messageLogged(
    const Ogre::String & message,
    Ogre::LogMessageLevel lml,
    bool maskDebug,
    const Ogre::String & logName,
    bool & skipThisMessage)
  {
    (void)maskDebug;
    (void)logName;
    if (!skipThisMessage) {
      if (lml >= min_lml) {
        switch (lml) {
          case Ogre::LogMessageLevel::LML_TRIVIAL:
            RVIZ_RENDERING_LOG_DEBUG(message.c_str());
            break;
          case Ogre::LogMessageLevel::LML_NORMAL:
            RVIZ_RENDERING_LOG_INFO(message.c_str());
            break;
          case Ogre::LogMessageLevel::LML_CRITICAL:
            RVIZ_RENDERING_LOG_ERROR(message.c_str());
            break;
          default:
            RVIZ_RENDERING_LOG_ERROR_STREAM("unknown Ogre log message level: " << lml);
        }
      }
    }
  }

  Ogre::LogMessageLevel min_lml;
};

namespace rviz_rendering
{

class OgreLoggingPrivate
{
public:
  CustomOgreLogListener custom_ogre_log_listener;
};

OgreLogging * OgreLogging::instance_ = nullptr;

OgreLogging *
OgreLogging::get()
{
  if (instance_ == 0) {
    instance_ = new OgreLogging();
  }
  return instance_;
}

OgreLogging::OgreLogging()
: dataPtr(std::make_unique<OgreLoggingPrivate>())
{
}

OgreLogging::~OgreLogging()
{
  Ogre::LogManager * log_manager = Ogre::LogManager::getSingletonPtr();
  delete log_manager;
  instance_ = nullptr;
}

void OgreLogging::useLogFile(const std::string & filename)
{
  preference_ = FileLogging;
  filename_ = filename;
}

void OgreLogging::useLogFileAndStandardOut(const std::string & filename)
{
  preference_ = StandardOut;
  filename_ = filename;
}

void OgreLogging::noLog()
{
  preference_ = NoLogging;
}

void OgreLogging::configureLogging()
{
  Ogre::LogManager * log_manager = Ogre::LogManager::getSingletonPtr();
  if (!log_manager) {
    // suppressing this memleak warning from cppcheck below
    // because this pointer is stored by Ogre internally
    log_manager = new Ogre::LogManager();
  }
  Ogre::Log * l = log_manager->createLog(filename_, false, false, (preference_ == NoLogging));
  l->addListener(&this->dataPtr->custom_ogre_log_listener);

  // Printing to standard out is what Ogre does if you don't do any LogManager calls.
  if (preference_ == StandardOut) {
    this->dataPtr->custom_ogre_log_listener.min_lml = Ogre::LML_NORMAL;
  }
  // cppcheck-suppress memleak
}

}  // namespace rviz_rendering
