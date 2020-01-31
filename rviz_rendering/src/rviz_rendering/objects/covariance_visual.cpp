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

#include "rviz_rendering/objects/covariance_visual.hpp"

#include <cmath>
#include <memory>
#include <sstream>
#include <tuple>
#include <vector>

#include <OgreEntity.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz_rendering/logging.hpp"
#include "rviz_rendering/objects/shape.hpp"

namespace rviz_rendering
{

namespace
{

Ogre::Real deg2rad(Ogre::Real degrees)
{
  return degrees * 3.14159265358979f / 180.0f;
}

// Local function to force the axis to be right handed for 3D. Taken from ecl_statistics
void makeRightHanded(Eigen::Matrix3d & eigenvectors, Eigen::Vector3d & eigenvalues)
{
  // Note that sorting of eigenvalues may end up with left-hand coordinate system.
  // So here we correctly sort it so that it does end up being right-handed and normalised.
  Eigen::Vector3d c0 = eigenvectors.block<3, 1>(0, 0);  c0.normalize();
  Eigen::Vector3d c1 = eigenvectors.block<3, 1>(0, 1);  c1.normalize();
  Eigen::Vector3d c2 = eigenvectors.block<3, 1>(0, 2);  c2.normalize();
  Eigen::Vector3d cc = c0.cross(c1);
  if (cc.dot(c2) < 0) {
    eigenvectors << c1, c0, c2;
    double e = eigenvalues[0];  eigenvalues[0] = eigenvalues[1];  eigenvalues[1] = e;
  } else {
    eigenvectors << c0, c1, c2;
  }
}

// Local function to force the axis to be right handed for 2D. Based on the one from ecl_statistics
void makeRightHanded(Eigen::Matrix2d & eigenvectors, Eigen::Vector2d & eigenvalues)
{
  // Note that sorting of eigenvalues may end up with left-hand coordinate system.
  // So here we correctly sort it so that it does end up being right-handed and normalised.
  Eigen::Vector3d c0;  c0.setZero();  c0.head<2>() = eigenvectors.col(0);  c0.normalize();
  Eigen::Vector3d c1;  c1.setZero();  c1.head<2>() = eigenvectors.col(1);  c1.normalize();
  Eigen::Vector3d cc = c0.cross(c1);
  if (cc[2] < 0) {
    eigenvectors << c1.head<2>(), c0.head<2>();
    double e = eigenvalues[0];  eigenvalues[0] = eigenvalues[1];  eigenvalues[1] = e;
  } else {
    eigenvectors << c0.head<2>(), c1.head<2>();
  }
}

struct Cov2DSolverParams
{
  static const size_t dimension = 2;
  using EigenvaluesType = Ogre::Vector2;
  using EigenvectorsType = Ogre::Matrix3;  // use as replacement for the non-existing Matrix2
  using ResultNumberType = Ogre::Real;
};
struct Cov3DSolverParams
{
  static const size_t dimension = 3;
  using EigenvaluesType = Ogre::Vector3;
  using EigenvectorsType = Ogre::Matrix3;
  using ResultNumberType = Ogre::Real;
};

template<typename SolverParams>
std::tuple<typename SolverParams::EigenvaluesType, typename SolverParams::EigenvectorsType>
diagonalizeCovariance(
  const Eigen::Matrix<double, SolverParams::dimension, SolverParams::dimension> & covariance)
{
  Eigen::Matrix<double, SolverParams::dimension, 1> eigenvalues =
    Eigen::Matrix<double, SolverParams::dimension, 1>::Identity();
  Eigen::Matrix<double, SolverParams::dimension, SolverParams::dimension> eigenvectors =
    Eigen::Matrix<double, SolverParams::dimension, SolverParams::dimension>::Zero();

  // NOTE: The SelfAdjointEigenSolver only
  // references the lower triangular part of the covariance matrix
  // TODO(anonymous): Should we use Eigen's pseudoEigenvectors() ?
  Eigen::SelfAdjointEigenSolver<
    Eigen::Matrix<double, SolverParams::dimension, SolverParams::dimension>>
  eigensolver(covariance);
  // Compute eigenvectors and eigenvalues
  bool covariance_valid = true;
  if (eigensolver.info() == Eigen::Success) {
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors = eigensolver.eigenvectors();

    if (eigenvalues.minCoeff() < 0) {
      RVIZ_RENDERING_LOG_WARNING(
        "Negative eigenvalue found for position. Is the "
        "covariance matrix correct (positive semidefinite)?");
      covariance_valid = false;
    }
  } else {
    RVIZ_RENDERING_LOG_WARNING(
      "failed to compute eigen vectors/values for position. Is the "
      "covariance matrix correct?");
    covariance_valid = false;
  }
  if (!covariance_valid) {
    eigenvalues = Eigen::Matrix<double, SolverParams::dimension, 1>::Zero();  // Setting the scale
    // to zero will hide it on the screen
    eigenvectors =
      Eigen::Matrix<double, SolverParams::dimension, SolverParams::dimension>::Identity();
  }

  // Be sure we have a right-handed orientation system
  makeRightHanded(eigenvectors, eigenvalues);

  typename SolverParams::EigenvaluesType result_values;
  typename SolverParams::EigenvectorsType result_vectors;
  for (auto i = 0u; i < SolverParams::dimension; ++i) {
    result_values[i] = static_cast<Ogre::Real>(eigenvalues[i]);
    for (auto j = 0u; j < SolverParams::dimension; ++j) {
      result_vectors[i][j] = static_cast<Ogre::Real>(eigenvectors(i, j));
    }
  }
  return std::make_tuple(result_values, result_vectors);
}

std::tuple<Ogre::Vector3, Ogre::Quaternion>
computeShapeScaleAndOrientation3D(const Eigen::Matrix3d & covariance)
{
  Cov3DSolverParams::EigenvaluesType eigenvalues;
  Cov3DSolverParams::EigenvectorsType eigenvectors;
  std::tie(eigenvalues, eigenvectors) =
    diagonalizeCovariance<Cov3DSolverParams>(covariance);

  // Define the rotation
  Ogre::Quaternion orientation;
  orientation.FromRotationMatrix(eigenvectors);

  // Define the scale. eigenvalues are the variances,
  // so we take the sqrt to draw the standard deviation
  Ogre::Vector3 scale(
    2.f * std::sqrt(eigenvalues[0]),
    2.f * std::sqrt(eigenvalues[1]),
    2.f * std::sqrt(eigenvalues[2]));
  return std::make_tuple(scale, orientation);
}

enum class Plane
{
  YZ_PLANE,  // normal is x-axis
  XZ_PLANE,  // normal is y-axis
  XY_PLANE   // normal is z-axis
};

std::tuple<Ogre::Vector3, Ogre::Quaternion>
computeShapeScaleAndOrientation2D(const Eigen::Matrix2d & covariance, Plane plane = Plane::XY_PLANE)
{
  Cov2DSolverParams::EigenvaluesType eigenvalues;
  Cov2DSolverParams::EigenvectorsType eigenvectors;
  std::tie(eigenvalues, eigenvectors) =
    diagonalizeCovariance<Cov2DSolverParams>(covariance);

  // Define the rotation and scale of the plane
  // The Eigenvalues are the variances. The scales are two times the standard
  // deviation. The scale of the missing dimension is set to zero.
  Ogre::Vector3 scale;
  Ogre::Quaternion orientation;
  switch (plane) {
    case Plane::YZ_PLANE:
      orientation.FromRotationMatrix(
        Ogre::Matrix3(
          1, 0, 0,
          0, eigenvectors[0][0], eigenvectors[0][1],
          0, eigenvectors[1][0], eigenvectors[1][1]));

      scale.x = 0;
      scale.y = 2.f * std::sqrt(eigenvalues[0]);
      scale.z = 2.f * std::sqrt(eigenvalues[1]);
      break;

    case Plane::XZ_PLANE:
      orientation.FromRotationMatrix(
        Ogre::Matrix3(
          eigenvectors[0][0], 0, eigenvectors[0][1],
          0, 1, 0,
          eigenvectors[1][0], 0, eigenvectors[1][1]));

      scale.x = 2.f * std::sqrt(eigenvalues[0]);
      scale.y = 0;
      scale.z = 2.f * std::sqrt(eigenvalues[1]);
      break;

    case Plane::XY_PLANE:
      orientation.FromRotationMatrix(
        Ogre::Matrix3(
          eigenvectors[0][0], eigenvectors[0][1], 0,
          eigenvectors[1][0], eigenvectors[1][1], 0,
          0, 0, 1));

      scale.x = 2.f * std::sqrt(eigenvalues[0]);
      scale.y = 2.f * std::sqrt(eigenvalues[1]);
      scale.z = 0;
  }
  return std::make_tuple(scale, orientation);
}

Ogre::Real radianScaleToMetricScaleBounded(Ogre::Real radian_scale, float max_degrees)
{
  if (radian_scale > 2.0 * deg2rad(max_degrees)) {
    return 2.0f * tanf(deg2rad(max_degrees));
  } else {
    return 2.0f * tanf(radian_scale * 0.5f);
  }
}

}  // namespace


CovarianceVisual::CovarianceVisual(
  Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node,
  bool is_local_rotation, bool is_visible, float pos_scale, float ori_scale, float ori_offset)
: scene_manager_(scene_manager),
  local_rotation_(is_local_rotation),
  pose_2d_(false),
  orientation_visible_(is_visible)
{
  // Main node of the visual
  root_node_ = parent_node->createChildSceneNode();
  // Node that will have the same orientation as the fixed frame.
  // Updated from the message on setCovariance()
  fixed_orientation_node_ = root_node_->createChildSceneNode();
  // Node to scale the position part of the covariance from the property value
  position_scale_node_ = fixed_orientation_node_->createChildSceneNode();
  // Node to be oriented and scaled from the message's covariance
  position_node_ = position_scale_node_->createChildSceneNode();
  position_shape_ = std::make_shared<rviz_rendering::Shape>(
    rviz_rendering::Shape::Sphere, scene_manager_, position_node_);

  // Node to scale the orientation part of the covariance. May be attached to both
  // the local (root) node or the fixed frame node.
  // May be re-attached later by setRotatingFrame()
  if (local_rotation_) {
    orientation_root_node_ = root_node_->createChildSceneNode();
  } else {
    orientation_root_node_ = fixed_orientation_node_->createChildSceneNode();
  }

  for (int i = 0; i < kNumOrientationShapes; i++) {
    // Node to position and orient the shape along the axis. One for each axis.
    orientation_offset_nodes_[i] = orientation_root_node_->createChildSceneNode();
    // Does not inherit scale from the parent.
    // This is needed to keep the cylinders with the same height.
    // The scale is set by setOrientationScale()
    orientation_offset_nodes_[i]->setInheritScale(false);

    if (i != kYaw2D) {
      orientation_shapes_[i] = std::make_shared<rviz_rendering::Shape>(
        rviz_rendering::Shape::Cylinder, scene_manager_, orientation_offset_nodes_[i]);
    } else {
      orientation_shapes_[i] = std::make_shared<rviz_rendering::Shape>(
        rviz_rendering::Shape::Cone, scene_manager_, orientation_offset_nodes_[i]);
    }

    // Initialize all current scales to 0
    current_orientation_scales_[i] = Ogre::Vector3::ZERO;
  }

  // Position the cylinders at position 1.0 in the respective axis, and perpendicular to the axis.
  // x-axis (roll)
  orientation_offset_nodes_[kRoll]->setPosition(Ogre::Vector3::UNIT_X);
  orientation_offset_nodes_[kRoll]->setOrientation(
    Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X) *
    Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Z) );
  // y-axis (pitch)
  orientation_offset_nodes_[kPitch]->setPosition(Ogre::Vector3(Ogre::Vector3::UNIT_Y) );
  orientation_offset_nodes_[kPitch]->setOrientation(
    Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y) );
  // z-axis (yaw)
  orientation_offset_nodes_[kYaw]->setPosition(Ogre::Vector3(Ogre::Vector3::UNIT_Z) );
  orientation_offset_nodes_[kYaw]->setOrientation(
    Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X) );
  // z-axis (yaw 2D)
  // NOTE: rviz use a cone defined by the file rviz_rendering/ogre_media/models/rviz_cone.mesh, and
  //       it's origin is not at the top of the cone. Since we want the top to be at the origin of
  //       the pose we need to use an offset here.
  // WARNING: This number was found by trial-and-error on rviz and it's not the correct
  //          one, so changes on scale are expected to cause the top of the cone to move
  //          from the pose origin, although it's only noticeable with big scales.
  // TODO(anonymous): Find the right value from the cone.mesh file, or implement a class that draws
  //        something like a 2D "pie slice" and use it instead of the cone.
  static const float cone_origin_to_top = 0.49115f;
  orientation_offset_nodes_[kYaw2D]->setPosition(cone_origin_to_top * Ogre::Vector3::UNIT_X);
  orientation_offset_nodes_[kYaw2D]->setOrientation(
    Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Z));

  // set initial visibility and scale
  // root node is always visible. The visibility will be updated on its children.
  root_node_->setVisible(true);
  setVisible(is_visible);
  setScales(pos_scale, ori_scale);
  setOrientationOffset(ori_offset);
}

CovarianceVisual::~CovarianceVisual()
{
  scene_manager_->destroySceneNode(position_node_);

  for (auto orientation_offset_node : orientation_offset_nodes_) {
    scene_manager_->destroySceneNode(orientation_offset_node);
  }

  scene_manager_->destroySceneNode(position_scale_node_);
  scene_manager_->destroySceneNode(fixed_orientation_node_);
  scene_manager_->destroySceneNode(root_node_);
}

void CovarianceVisual::updateUserData(CovarianceUserData user_data)
{
  setPositionColor(user_data.position_color);
  setPositionScale(user_data.position_scale);

  if (user_data.orientation_color_style == Unique) {
    setOrientationColor(user_data.orientation_color);
  } else {
    setOrientationColorToRGB(user_data.orientation_color.a);
  }
  setOrientationOffset(user_data.orientation_offset);
  setOrientationScale(user_data.orientation_scale);

  if (!user_data.visible) {
    setVisible(false);
  } else {
    setPositionVisible(user_data.position_visible);
    setOrientationVisible(user_data.orientation_visible);
  }

  setRotatingFrame(user_data.orientation_frame == Local);
}

void CovarianceVisual::setCovariance(
  const Ogre::Quaternion & pose_orientation,
  const std::array<double, 36> & covariances)
{
  // check for NaN in covariance
  for (unsigned i = 0; i < 3; ++i) {
    if (std::isnan(covariances[i])) {
      RVIZ_RENDERING_LOG_WARNING("covariance contains NaN");
      return;
    }
  }

  // The 3rd, 4th, and 5th diagonal entries are empty for 2D case
  pose_2d_ = (covariances[14] <= 0 && covariances[21] <= 0 && covariances[28] <= 0);

  updateOrientationVisibility();

  // store orientation in Ogre structure
  // Set the orientation of the fixed node. Since this node is attached to the root node,
  // it's orientation will be the inverse of pose's orientation.
  fixed_orientation_node_->setOrientation(pose_orientation.Inverse());
  // Map covariance to a Eigen::Matrix
  Eigen::Map<const Eigen::Matrix<double, 6, 6>> covariance(covariances.data());

  updatePosition(covariance);
  if (!pose_2d_) {
    updateOrientation(covariance, kRoll);
    updateOrientation(covariance, kPitch);
    updateOrientation(covariance, kYaw);
  } else {
    updateOrientation(covariance, kYaw2D);
  }
}

void CovarianceVisual::updatePosition(const Eigen::Matrix6d & covariance)
{
  // Compute shape and orientation for the position part of covariance
  Ogre::Vector3 shape_scale;
  Ogre::Quaternion shape_orientation;
  if (pose_2d_) {
    std::tie(shape_scale, shape_orientation) =
      computeShapeScaleAndOrientation2D(covariance.topLeftCorner<2, 2>(), Plane::XY_PLANE);
    // Make the scale in z minimal for better visualization
    shape_scale.z = 0.001f;
  } else {
    std::tie(shape_scale, shape_orientation) =
      computeShapeScaleAndOrientation3D(covariance.topLeftCorner<3, 3>());
  }
  // Rotate and scale the position scene node
  position_node_->setOrientation(shape_orientation);
  if (!shape_scale.isNaN()) {
    position_node_->setScale(shape_scale);
  } else {
    RVIZ_RENDERING_LOG_WARNING_STREAM("position shape_scale contains NaN: " << shape_scale);
  }
}

void CovarianceVisual::updateOrientation(const Eigen::Matrix6d & covariance, ShapeIndex index)
{
  Ogre::Vector3 shape_scale;
  Ogre::Quaternion shape_orientation;
  // Compute shape and orientation for the orientation shape
  if (pose_2d_) {
    // We should only enter on this scope if the index is kYaw2D
    assert(index == kYaw2D);
    // 2D poses only depend on yaw.
    shape_scale.x = static_cast<Ogre::Real>(2.0 * std::sqrt(covariance(5, 5)));
    // To display the cone shape properly the scale along y-axis has to be one.
    shape_scale.y = 1.0f;
    // Give a minimal height for the cone for better visualization
    shape_scale.z = 0.001f;
    // Store the computed scale to be used if the user change the scale
    current_orientation_scales_[index] = shape_scale;
    // Apply the current scale factor
    shape_scale.x *= current_orientation_scale_factor_;
    // The scale on x means twice the standard deviation, but _in radians_.
    // So we need to convert it to the linear scale of the shape using tan().
    // Also, we bound the maximum std
    shape_scale.x = radianScaleToMetricScaleBounded(shape_scale.x, kMaxDegrees);
  } else {
    assert(index != kYaw2D);

    // Get the correct sub-matrix based on the index
    Eigen::Matrix2d covarianceAxis;
    if (index == kRoll) {
      covarianceAxis = covariance.bottomRightCorner<2, 2>();
    } else if (index == kPitch) {
      covarianceAxis << covariance(3, 3), covariance(3, 5), covariance(5, 3), covariance(5, 5);
    } else if (index == kYaw) {
      covarianceAxis = covariance.block<2, 2>(3, 3);
    }

    // NOTE: The cylinder mesh is oriented along its y axis,
    // thus we want to flat it out into the XZ plane
    std::tie(shape_scale, shape_orientation) =
      computeShapeScaleAndOrientation2D(covarianceAxis, Plane::XZ_PLANE);
    // Give a minimal height for the cylinder for better visualization
    shape_scale.y = 0.001f;
    // Store the computed scale to be used if the user change the scale
    current_orientation_scales_[index] = shape_scale;
    // Apply the current scale factor
    shape_scale.x *= current_orientation_scale_factor_;
    shape_scale.z *= current_orientation_scale_factor_;
    // The computed scale is equivalent to twice the standard deviation _in radians_.
    // So we need to convert it to the linear scale of the shape using tan().
    // Also, we bound the maximum std.
    shape_scale.x = radianScaleToMetricScaleBounded(shape_scale.x, kMaxDegrees);
    shape_scale.z = radianScaleToMetricScaleBounded(shape_scale.z, kMaxDegrees);
  }

  // Rotate and scale the scene node of the orientation part
  orientation_shapes_[index]->setOrientation(shape_orientation);
  if (!shape_scale.isNaN()) {
    orientation_shapes_[index]->setScale(shape_scale);
  } else {
    RVIZ_RENDERING_LOG_WARNING_STREAM("orientation shape_scale contains NaN: " << shape_scale);
  }
}

void CovarianceVisual::setScales(float pos_scale, float ori_scale)
{
  setPositionScale(pos_scale);
  setOrientationScale(ori_scale);
}

void CovarianceVisual::setPositionScale(float pos_scale)
{
  if (pose_2d_) {
    position_scale_node_->setScale(pos_scale, pos_scale, 1.0);
  } else {
    position_scale_node_->setScale(pos_scale, pos_scale, pos_scale);
  }
}

void CovarianceVisual::setOrientationOffset(float ori_offset)
{
  // Scale the orientation root node to position the shapes along the axis
  orientation_root_node_->setScale(ori_offset, ori_offset, ori_offset);
  // The scale the offset_nodes as well so the displayed shape represents a 1-sigma
  // standard deviation when displayed with an scale of 1.0
  // NOTE: We only want to change the scales of the dimensions that represent the
  //       orientation covariance. The other dimensions are set to 1.0.
  for (int i = 0; i < kNumOrientationShapes; i++) {
    if (i == kYaw2D) {
      // For 2D, the angle is only encoded on x,
      // but we also scale on y to put the top of the cone at the pose origin
      orientation_offset_nodes_[i]->setScale(ori_offset, ori_offset, 1.0);
    } else {
      // For 3D, the angle covariance is encoded on x and z dimensions
      orientation_offset_nodes_[i]->setScale(ori_offset, 1.0, ori_offset);
    }
  }
}

void CovarianceVisual::setOrientationScale(float ori_scale)
{
  // Here we update the current scale factor, apply it to the current scale _in radians_,
  // convert it to meters and apply to the shape scale. Note we have different invariant
  // scales in the 3D and in 2D.
  current_orientation_scale_factor_ = ori_scale;
  for (int i = 0; i < kNumOrientationShapes; i++) {
    // Recover the last computed scale
    Ogre::Vector3 shape_scale = current_orientation_scales_[i];
    if (i == kYaw2D) {
      // Changes in scale in 2D only affects the x dimension
      // Apply the current scale factor
      shape_scale.x *= current_orientation_scale_factor_;
      // Convert from radians to meters
      shape_scale.x = radianScaleToMetricScaleBounded(shape_scale.x, kMaxDegrees);
    } else {
      // Changes in scale in 3D only affects the x and z dimensions
      // Apply the current scale factor
      shape_scale.x *= current_orientation_scale_factor_;
      shape_scale.z *= current_orientation_scale_factor_;
      // Convert from radians to meters
      shape_scale.x = radianScaleToMetricScaleBounded(shape_scale.x, kMaxDegrees);
      shape_scale.z = radianScaleToMetricScaleBounded(shape_scale.z, kMaxDegrees);
    }
    // Apply the new scale
    orientation_shapes_[i]->setScale(shape_scale);
  }
}

void CovarianceVisual::setPositionColor(const Ogre::ColourValue & c)
{
  position_shape_->setColor(c);
}

void CovarianceVisual::setOrientationColor(const Ogre::ColourValue & c)
{
  for (auto orientation_shape : orientation_shapes_) {
    orientation_shape->setColor(c);
  }
}

void CovarianceVisual::setOrientationColorToRGB(float alpha)
{
  orientation_shapes_[kRoll]->setColor(Ogre::ColourValue(1.0, 0.0, 0.0, alpha));
  orientation_shapes_[kPitch]->setColor(Ogre::ColourValue(0.0, 1.0, 0.0, alpha));
  orientation_shapes_[kYaw]->setColor(Ogre::ColourValue(0.0, 0.0, 1.0, alpha));
  orientation_shapes_[kYaw2D]->setColor(Ogre::ColourValue(0.0, 0.0, 1.0, alpha));
}

void CovarianceVisual::setVisible(bool visible)
{
  setPositionVisible(visible);
  setOrientationVisible(visible);
}

void CovarianceVisual::setPositionVisible(bool visible)
{
  position_node_->setVisible(visible);
}

void CovarianceVisual::setOrientationVisible(bool visible)
{
  orientation_visible_ = visible;
  updateOrientationVisibility();
}

void CovarianceVisual::updateOrientationVisibility()
{
  orientation_offset_nodes_[kRoll]->setVisible(orientation_visible_ && !pose_2d_);
  orientation_offset_nodes_[kPitch]->setVisible(orientation_visible_ && !pose_2d_);
  orientation_offset_nodes_[kYaw]->setVisible(orientation_visible_ && !pose_2d_);
  orientation_offset_nodes_[kYaw2D]->setVisible(orientation_visible_ && pose_2d_);
}

void CovarianceVisual::setPosition(const Ogre::Vector3 & position)
{
  root_node_->setPosition(position);
}

void CovarianceVisual::setOrientation(const Ogre::Quaternion & orientation)
{
  root_node_->setOrientation(orientation);
}

void CovarianceVisual::setRotatingFrame(bool is_local_rotation)
{
  if (local_rotation_ == is_local_rotation) {
    return;
  }

  local_rotation_ = is_local_rotation;

  if (local_rotation_) {
    root_node_->addChild(fixed_orientation_node_->removeChild(orientation_root_node_));
  } else {
    fixed_orientation_node_->addChild(root_node_->removeChild(orientation_root_node_));
  }
}

Ogre::AxisAlignedBox CovarianceVisual::getPositionBoundingBox()
{
  return position_shape_->getEntity()->getWorldBoundingBox();
}

std::vector<Ogre::AxisAlignedBox> CovarianceVisual::getOrientationBoundingBoxes()
{
  std::vector<Ogre::AxisAlignedBox> aabbs;
  aabbs.push_back(orientation_shapes_[kRoll]->getEntity()->getWorldBoundingBox());
  aabbs.push_back(orientation_shapes_[kPitch]->getEntity()->getWorldBoundingBox());
  aabbs.push_back(orientation_shapes_[kYaw]->getEntity()->getWorldBoundingBox());
  return aabbs;
}

}  // namespace rviz_rendering
