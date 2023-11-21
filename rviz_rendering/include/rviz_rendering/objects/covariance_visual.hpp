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
#include <vector>

// GCC 11 has a false positive warning about uninitialized variables in Eigen.  There is an open
// issue about it at https://gitlab.com/libeigen/eigen/-/issues/2304 .  Just disable the warning
// for Eigen for now.
// Also Version 3.4.0 of Eigen in Ubuntu 22.04 has a bug that causes -Wclass-memaccess warnings on
// aarch64.  Upstream Eigen has already fixed this in
// https://gitlab.com/libeigen/eigen/-/merge_requests/645 .  The Debian fix for this is in
// https://salsa.debian.org/science-team/eigen3/-/merge_requests/1 .
// However, it is not clear that that fix is going to make it into Ubuntu 22.04 before it
// freezes, so disable the warning here.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#elif defined(_WIN32)
#pragma warning(push)
#pragma warning(disable:4996)
#endif
#include <Eigen/Dense>
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#elif defined(_WIN32)
#pragma warning(pop)
#endif

#include <OgreVector.h>
#include <OgreColourValue.h>

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

enum RVIZ_RENDERING_PUBLIC Frame
{
  Local,
  Fixed,
};

enum RVIZ_RENDERING_PUBLIC ColorStyle
{
  Unique,
  RGB,
};

struct RVIZ_RENDERING_PUBLIC CovarianceUserData
{
  bool visible;

  bool position_visible;
  Ogre::ColourValue position_color;
  float position_scale;

  bool orientation_visible;
  Frame orientation_frame;
  ColorStyle orientation_color_style;
  Ogre::ColourValue orientation_color;
  float orientation_offset;
  float orientation_scale;
};

/**
 * \class CovarianceVisual
 * \brief CovarianceVisual consisting in a ellipse for position and 2D ellipses along the axis for orientation.
 */
class CovarianceVisual
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
    Ogre::SceneManager * scene_manager,
    Ogre::SceneNode * parent_node,
    bool is_local_rotation = false,
    bool is_visible = true,
    float pos_scale = 1.0f,
    float ori_scale = 0.1f,
    float ori_offset = 0.1f);

public:
  RVIZ_RENDERING_PUBLIC
  virtual ~CovarianceVisual();

  RVIZ_RENDERING_PUBLIC
  void updateUserData(CovarianceUserData user_data);

  /** @brief Set the covariance.
   *
   * This effectively changes the orientation and scale of position and orientation
   * covariance shapes
   */
  RVIZ_RENDERING_PUBLIC
  virtual void setCovariance(
    const Ogre::Quaternion & pose_orientation,
    const std::array<double, 36> & covariances);

  /**
   * \brief Sets position of the frame this covariance is attached
   */
  RVIZ_RENDERING_PUBLIC
  void setPosition(const Ogre::Vector3 & position);

  /**
   * \brief Sets orientation of the frame this covariance is attached
   */
  RVIZ_RENDERING_PUBLIC
  void setOrientation(const Ogre::Quaternion & orientation);

  /**
   * \brief Sets visibility of this covariance
   *
   * Convenience method that sets visibility of both position and orientation parts.
   */
  RVIZ_RENDERING_PUBLIC
  virtual void setVisible(bool visible);

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

  RVIZ_RENDERING_PUBLIC
  Ogre::AxisAlignedBox getPositionBoundingBox();

  RVIZ_RENDERING_PUBLIC
  std::vector<Ogre::AxisAlignedBox> getOrientationBoundingBoxes();

private:
  /**
   * \brief Set the position and orientation scales for this covariance
   *
   * @param pos_scale Scale of the position covariance
   * @param ori_scale Scale of the orientation covariance
   */
  void setScales(float pos_scale, float ori_scale);

  void setPositionScale(float pos_scale);

  void setOrientationOffset(float ori_offset);

  void setOrientationScale(float ori_scale);

  void setPositionColor(const Ogre::ColourValue & color);

  void setOrientationColor(const Ogre::ColourValue & color);

  void setOrientationColorToRGB(float alpha);

  /**
   * \brief Get the shape used to display position covariance
   * @return the shape used to display position covariance
   */
  std::shared_ptr<rviz_rendering::Shape> getPositionShape() {return position_shape_;}

  /**
   * \brief Sets visibility of the position part of this covariance
   */
  virtual void setPositionVisible(bool visible);

  /**
   * \brief Sets visibility of the orientation part of this covariance
   */
  virtual void setOrientationVisible(bool visible);

  /**
   * \brief Sets which frame to attach the covariance of the orientation
   */
  virtual void setRotatingFrame(bool use_rotating_frame);

  void updatePosition(const Eigen::Matrix6d & covariance);

  void updateOrientation(const Eigen::Matrix6d & covariance, ShapeIndex index);

  void updateOrientationVisibility();

  Ogre::SceneManager * scene_manager_;

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
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OBJECTS__COVARIANCE_VISUAL_HPP_
