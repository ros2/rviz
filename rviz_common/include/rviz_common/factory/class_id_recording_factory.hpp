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

#ifndef RVIZ_COMMON__FACTORY__CLASS_ID_RECORDING_FACTORY_HPP_
#define RVIZ_COMMON__FACTORY__CLASS_ID_RECORDING_FACTORY_HPP_

#include "./factory.hpp"

namespace rviz_common
{

template<class Type>
/// Templated factory which informs objects created by it what their class identifier string was.
/**
 * calls a setClassId() function on any instances created by a protected
 * makeRaw() function (pure virtual in this class).
 */
class ClassIdRecordingFactory : public Factory
{
public:
  /// Instantiate and return a instance of a subclass of Type using makeRaw().
  /**
   * If make() returns nullptr and error_return is not nullptr,
   * *error_return will be set.
   * On success, *error_return will not be changed.
   *
   * \param class_id A string identifying the class uniquely among classes of
   *   its parent class, e.g. rviz::GridDisplay might be 'rviz/Grid'.
   * \param error_return If non-nullptr and there is an error,
   *   *error_return is set to a description of the problem.
   * \return A new instance of the class identified by class_id, or
   *   nullptr if there was an error.
   */
  virtual Type * make(const QString & class_id, QString * error_return = nullptr)
  {
    Type * obj = makeRaw(class_id, error_return);
    if (obj != nullptr) {
      obj->setClassId(class_id);
      obj->setDescription(getPluginInfo(class_id).description);
    }
    return obj;
  }

protected:
  virtual Type * makeRaw(const QString & class_id, QString * error_return = nullptr) = 0;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__FACTORY__CLASS_ID_RECORDING_FACTORY_HPP_
