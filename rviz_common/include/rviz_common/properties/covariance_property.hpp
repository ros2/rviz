/*
 * Copyright (c) 2017, Ellon Paiva Mendes @ LAAS-CNRS
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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
 *     * Neither the name of the copyright holders nor the names of its
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

#ifndef RVIZ_COMMON__PROPERTIES__COVARIANCE_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__COVARIANCE_PROPERTY_HPP_

#include <deque>
#include <memory>

#include <OgreColourValue.h>

#include <QColor>  // NOLINT cpplint cannot handle include order here

#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace rviz_rendering
{
struct CovarianceUserData;
}

namespace rviz_common
{
namespace properties
{
class Property;
class ColorProperty;
class FloatProperty;
class EnumProperty;


/** @brief Property specialized for covariance visuals. */
class RVIZ_COMMON_PUBLIC CovarianceProperty : public rviz_common::properties::BoolProperty
{
  Q_OBJECT

public:
  explicit CovarianceProperty(
    const QString & name = "Covariance",
    bool default_value = false,
    const QString & description = QString(),
    Property * parent = nullptr,
    const char * changed_slot = nullptr);

  ~CovarianceProperty() override = default;

  /**
   * @brief Return all property data in an object understood by covariance_visual
   */
  rviz_rendering::CovarianceUserData getUserData();

private Q_SLOTS:
  void updateColorStyleChoice();

private:
  BoolProperty * position_property_;
  ColorProperty * position_color_property_;
  FloatProperty * position_alpha_property_;
  FloatProperty * position_scale_property_;
  BoolProperty * orientation_property_;
  EnumProperty * orientation_frame_property_;
  EnumProperty * orientation_colorstyle_property_;
  ColorProperty * orientation_color_property_;
  FloatProperty * orientation_alpha_property_;
  FloatProperty * orientation_offset_property_;
  FloatProperty * orientation_scale_property_;
};


}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__COVARIANCE_PROPERTY_HPP_
