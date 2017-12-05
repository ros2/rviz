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

#ifndef RVIZ_COMMON__FAILED_VIEW_CONTROLLER_HPP_
#define RVIZ_COMMON__FAILED_VIEW_CONTROLLER_HPP_

#include "rviz_common/view_controller.hpp"

namespace rviz_common
{

/// Represents a ViewController class which tried and failed to instantiate.
/**
 * FailedViewController stores the class id which it was supposed to be, and
 * an error message describing the failure.
 *
 * The load() and save() functions work together to ensure that loaded
 * configuration data is saved out again without modification.  This
 * ensures that running rviz with a missing plugin library won't
 * damage config files which refer to it.
 */
class FailedViewController : public ViewController
{
public:
  FailedViewController(const QString & desired_class_id, const QString & error_message);

  QString getDescription() const override;

  void onActivate() override;

  void load(const Config & config) override;

  void save(Config config) const override;

  void lookAt(const Ogre::Vector3 & point) override;

  void reset() override {}

private:
  Config saved_config_;
  QString error_message_;
};

}  // end namespace rviz_common

#endif  // RVIZ_COMMON__FAILED_VIEW_CONTROLLER_HPP_
