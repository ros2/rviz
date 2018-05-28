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

#ifndef RVIZ_RENDERING__OBJECTS__COVARIANCE_VISUAL_HPP_
#define RVIZ_RENDERING__OBJECTS__COVARIANCE_VISUAL_HPP_

#include <array>
#include <memory>

#include <Eigen/Dense>  // NOLINT: cpplint cannot handle correct include here

#include <OgreVector3.h>
#include <OgreColourValue.h>

#include "rviz_rendering/objects/object.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
class Any;
}

namespace Eigen
{
using Matrix6d = Matrix<double, 6, 6>;
}

namespace rviz_rendering
{

/**
 * \class CovarianceVisual
 * \brief CovarianceVisual consisting in a ellipse for position and 2D ellipses along the axis for orientation.
 */
class CovarianceVisual : public rviz_rendering::Object
{
public:
  enum ShapeIndex
  {
    kRoll = 0,
    kPitch = 1,
    kYaw = 2,
    kYaw2D = 3,
    kNumOrientationShapes
  };

  /**
   * \brief Constructor - to be used in conjunction with covariance property
   *
   *
   * @param scene_manager The scene manager to use to construct any necessary objects
   * @param parent_object A rviz object that this covariance will be attached.
   * @param is_local_rotation Initial attachment of the rotation part
   * @param is_visible Initial visibility
   * @param pos_scale Scale of the position covariance
   * @param ori_scale Scale of the orientation covariance
   */
  RVIZ_RENDERING_PUBLIC
  CovarianceVisual(
    Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node,
    bool is_local_rotation, bool is_visible = true, float pos_scale = 1.0f, float ori_scale = 0.1f,
    float ori_offset = 0.1f);

public:
  RVIZ_RENDERING_PUBLIC
  ~CovarianceVisual() override;

  /**
   * \brief Set the position and orientation scales for this covariance
   *
   * @param pos_scale Scale of the position covariance
   * @param ori_scale Scale of the orientation covariance
   */
  RVIZ_RENDERING_PUBLIC
  void setScales(float pos_scale, float ori_scale);

  RVIZ_RENDERING_PUBLIC
  void setPositionScale(float pos_scale);

  RVIZ_RENDERING_PUBLIC
  void setOrientationOffset(float ori_offset);

  RVIZ_RENDERING_PUBLIC
  void setOrientationScale(float ori_scale);

  /**
   * \brief Set the color of the position covariance. Values are in the range [0, 1]
   *
   * @param r Red component
   * @param g Green component
   * @param b Blue component
   */
  RVIZ_RENDERING_PUBLIC
  virtual void setPositionColor(float r, float g, float b, float a);

  RVIZ_RENDERING_PUBLIC
  void setPositionColor(const Ogre::ColourValue & color);

  /**
   * \brief Set the color of the orientation covariance. Values are in the range [0, 1]
   *
   * @param r Red component
   * @param g Green component
   * @param b Blue component
   */
  RVIZ_RENDERING_PUBLIC
  virtual void setOrientationColor(float r, float g, float b, float a);

  RVIZ_RENDERING_PUBLIC
  void setOrientationColor(const Ogre::ColourValue & color);

  RVIZ_RENDERING_PUBLIC
  void setOrientationColorToRGB(float alpha);

  /** @brief Set the covariance.
   *
   * This effectively changes the orientation and scale of position and orientation
   * covariance shapes
   */
  RVIZ_RENDERING_PUBLIC
  virtual void setCovariance(
    const Ogre::Quaternion & pose_orientation,
    const std::array<double, 36> & covariances);

  RVIZ_RENDERING_PUBLIC
  virtual const Ogre::Vector3 & getPositionCovarianceScale();

  RVIZ_RENDERING_PUBLIC
  virtual const Ogre::Quaternion & getPositionCovarianceOrientation();

  /**
   * \brief Get the root scene node of the position part of this covariance
   * @return the root scene node of the position part of this covariance
   */
  RVIZ_RENDERING_PUBLIC
  Ogre::SceneNode * getPositionSceneNode() {return position_scale_node_;}

  /**
   * \brief Get the root scene node of the orientation part of this covariance
   * @return the root scene node of the orientation part of this covariance
   */
  RVIZ_RENDERING_PUBLIC
  Ogre::SceneNode * getOrientationSceneNode() {return orientation_root_node_;}

  /**
   * \brief Get the shape used to display position covariance
   * @return the shape used to display position covariance
   */
  RVIZ_RENDERING_PUBLIC
  std::shared_ptr<rviz_rendering::Shape> getPositionShape() {return position_shape_;}

  /**
   * \brief Get the shape used to display orientation covariance in an especific axis
   * @return the shape used to display orientation covariance in an especific axis
   */
  RVIZ_RENDERING_PUBLIC
  std::shared_ptr<rviz_rendering::Shape> getOrientationShape(ShapeIndex index);

  /**
   * \brief Sets user data on all ogre objects we own
   */
  RVIZ_RENDERING_PUBLIC
  void setUserData(const Ogre::Any & data) override;

  /**
   * \brief Sets visibility of this covariance
   *
   * Convenience method that sets visibility of both position and orientation parts.
   */
  RVIZ_RENDERING_PUBLIC
  virtual void setVisible(bool visible);

  /**
   * \brief Sets visibility of the position part of this covariance
   */
  RVIZ_RENDERING_PUBLIC
  virtual void setPositionVisible(bool visible);

  /**
   * \brief Sets visibility of the orientation part of this covariance
   */
  RVIZ_RENDERING_PUBLIC
  virtual void setOrientationVisible(bool visible);

  /**
   * \brief Sets position of the frame this covariance is attached
   */
  RVIZ_RENDERING_PUBLIC
  void setPosition(const Ogre::Vector3 & position) override;

  /**
   * \brief Sets orientation of the frame this covariance is attached
   */
  RVIZ_RENDERING_PUBLIC
  void setOrientation(const Ogre::Quaternion & orientation) override;

  /**
   * \brief Sets which frame to attach the covariance of the orientation
   */
  RVIZ_RENDERING_PUBLIC
  virtual void setRotatingFrame(bool use_rotating_frame);

private:
  void updatePosition(const Eigen::Matrix6d & covariance);

  void updateOrientation(const Eigen::Matrix6d & covariance, ShapeIndex index);

  void updateOrientationVisibility();

  Ogre::SceneNode * root_node_;
  Ogre::SceneNode * fixed_orientation_node_;
  Ogre::SceneNode * position_scale_node_;
  Ogre::SceneNode * position_node_;

  Ogre::SceneNode * orientation_root_node_;
  std::array<Ogre::SceneNode *, kNumOrientationShapes> orientation_offset_nodes_;

  /// Ellipse used for the position covariance
  std::shared_ptr<rviz_rendering::Shape> position_shape_;
  /// Cylinders and cone used for the orientation covariance
  std::array<std::shared_ptr<rviz_rendering::Shape>, kNumOrientationShapes> orientation_shapes_;

  bool local_rotation_;

  bool pose_2d_;

  bool orientation_visible_;  ///< If the orientation component is visible.

  std::array<Ogre::Vector3, kNumOrientationShapes> current_orientation_scales_;
  float current_orientation_scale_factor_;

  static constexpr float kMaxDegrees = 89.0f;

private:
  // Hide Object methods we don't want to expose
  // NOTE: Apparently we still need to define them...
  void setScale(const Ogre::Vector3 & scale) override
  {
    (void) scale;
  }

  void setColor(float r, float g, float b, float a) override
  {
    (void) r;
    (void) g;
    (void) b;
    (void) a;
  }

  const Ogre::Vector3 & getPosition() override;

  const Ogre::Quaternion & getOrientation() override;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OBJECTS__COVARIANCE_VISUAL_HPP_
