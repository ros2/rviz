/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__ROBOT__ROBOT_LINK_HPP_
#define RVIZ_DEFAULT_PLUGINS__ROBOT__ROBOT_LINK_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#ifndef Q_MOC_RUN

#include <OgreVector.h>
#include <OgreQuaternion.h>
#include <OgreAny.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

#endif

#include <QObject>

#include <urdf/model.h>  // can be replaced later by urdf_model/types.h
#include <urdf_model/pose.h>

#include "rviz_rendering/objects/object.hpp"
#include "rviz_common/interaction/forwards.hpp"

#include "rviz_default_plugins/robot/robot_element_base_class.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class Entity;
class SubEntity;
class SceneNode;
class Quaternion;
class Any;
class RibbonTrail;
}  // namespace Ogre

namespace rviz_rendering
{
class Shape;
class Axes;
}  // namespace rviz_rendering

namespace rviz_common
{
namespace properties
{
class FloatProperty;
class Property;
class BoolProperty;
class QuaternionProperty;
class VectorProperty;
}  // namespace properties
class DisplayContext;
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace robot
{

class Robot;
class RobotLinkSelectionHandler;
class RobotJoint;
typedef std::shared_ptr<RobotLinkSelectionHandler> RobotLinkSelectionHandlerPtr;


/**
 * \struct RobotLink
 * \brief Contains any data we need from a link in the robot.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC RobotLink : public RobotElementBaseClass
{
  Q_OBJECT

public:
  RobotLink(
    Robot * robot,
    const urdf::LinkConstSharedPtr & link,
    const std::string & parent_joint_name,
    bool visual,
    bool collision,
    bool mass,
    bool inertia);
  ~RobotLink() override;

  virtual void setRobotAlpha(float a);

  virtual void setTransforms(
    const Ogre::Vector3 & visual_position, const Ogre::Quaternion & visual_orientation,
    const Ogre::Vector3 & collision_position, const Ogre::Quaternion & collision_orientation);

  void setToErrorMaterial();
  void setToNormalMaterial();

  void setColor(float red, float green, float blue);
  void unsetColor();

  /// set whether the link is selectable.  If false objects behind/inside the
  /// link can be selected/manipulated.  Returns old value.
  bool setSelectable(bool selectable);
  bool getSelectable();

  bool hasGeometry() const;

  /* If set to true, the link will only render to the depth channel
   * and be in render group 0, so it is rendered before anything else.
   * Thus, it will occlude other objects without being visible.
   */
  void setOnlyRenderDepth(bool onlyRenderDepth);
  bool getOnlyRenderDepth() const {return only_render_depth_;}

  // access
  const std::string & getName() const {return name_;}
  const std::string & getParentJointName() const {return parent_joint_name_;}
  const std::vector<std::string> & getChildJointNames() const {return child_joint_names_;}
  rviz_common::properties::Property * getLinkProperty() const {return robot_element_property_;}
  Ogre::SceneNode * getVisualNode() const {return visual_node_;}
  Ogre::SceneNode * getCollisionNode() const {return collision_node_;}
  Robot * getRobot() const {return robot_;}
  const std::string getGeometryErrors() const;

  // get the meshes vector to be used in robot_test.cpp
  std::vector<Ogre::Entity *> getVisualMeshes() {return visual_meshes_;}
  std::vector<Ogre::Entity *> getCollisionMeshes() {return collision_meshes_;}

public Q_SLOTS:
  /** @brief Update the visibility of the link elements: visual mesh,
   * collision mesh, trail, and axes.
   *
   * Called by Robot when changing visual and collision visibilities,
   * since each link may be enabled or disabled. */
  void updateVisibility();

private Q_SLOTS:
  void updateAlpha();
  void updateTrail();

private:
  void setProperties(const urdf::LinkConstSharedPtr & link);
  void createDescription(const urdf::LinkConstSharedPtr & link);
  void setRenderQueueGroup(Ogre::uint8 group);
  bool getEnabled() const override;
  Ogre::Entity * createEntityForGeometryElement(
    const urdf::LinkConstSharedPtr & link,
    const urdf::Geometry & geom, const urdf::Pose & origin,
    std::string material_name, Ogre::SceneNode * scene_node);
  void assignMaterialsToEntities(
    const urdf::LinkConstSharedPtr & link,
    const std::string & material_name,
    const Ogre::Entity * entity);
  Ogre::MaterialPtr getMaterialForLink(
    const urdf::LinkConstSharedPtr & link, std::string material_name = "");
  urdf::VisualSharedPtr getVisualWithMaterial(
    const urdf::LinkConstSharedPtr & link, const std::string & material_name) const;
  void loadMaterialFromTexture(
    Ogre::MaterialPtr & material_for_link, const urdf::VisualSharedPtr & visual) const;

  void createCollision(const urdf::LinkConstSharedPtr & link);

  void addError(const char * format, ...);

  void createVisual(const urdf::LinkConstSharedPtr & link);
  void createMass(const urdf::LinkConstSharedPtr & link);
  void createInertia(const urdf::LinkConstSharedPtr & link);
  void createSelection();

  template<typename T>
  void createVisualizable(
    const urdf::LinkConstSharedPtr & link,
    std::vector<Ogre::Entity *> & meshes_vector,
    const std::vector<T> & visualizables_array,
    const T & visualizable_element,
    Ogre::SceneNode * scene_node)
  {
    bool valid_visualizable_found = false;

    for (const auto & vector_element : visualizables_array) {
      T link_visual_element = vector_element;
      if (link_visual_element && link_visual_element->geometry) {
        Ogre::Entity * mesh = createEntityForGeometryElement(
          link, *link_visual_element->geometry, link_visual_element->origin, "", scene_node);
        if (mesh) {
          meshes_vector.push_back(mesh);
          valid_visualizable_found = true;
        }
      }
    }

    if (!valid_visualizable_found && visualizable_element && visualizable_element->geometry) {
      Ogre::Entity * mesh = createEntityForGeometryElement(
        link, *visualizable_element->geometry, visualizable_element->origin, "", scene_node);
      if (mesh) {
        meshes_vector.push_back(mesh);
      }
    }
  }

protected:
  Ogre::SceneManager * scene_manager_;
  rviz_common::DisplayContext * context_;

  std::string parent_joint_name_;
  std::vector<std::string> child_joint_names_;

  // properties
  rviz_common::properties::Property * trail_property_;
  rviz_common::properties::FloatProperty * alpha_property_;

private:
  typedef std::map<Ogre::SubEntity *, Ogre::MaterialPtr> M_SubEntityToMaterial;
  M_SubEntityToMaterial materials_;
  Ogre::MaterialPtr default_material_;
  std::string default_material_name_;

  std::vector<Ogre::Entity *> visual_meshes_;    ///< The entities representing the
///< visual mesh of this link (if they exist)
  std::vector<Ogre::Entity *> collision_meshes_;  ///< The entities representing the
///< collision mesh of this link (if they exist)

  Ogre::SceneNode * visual_node_;          ///< The scene node the visual meshes are attached to
  Ogre::SceneNode * collision_node_;       ///< The scene node the collision meshes are attached to
  Ogre::SceneNode * mass_node_;            ///< The scene node the visual meshes are attached to
  Ogre::SceneNode * inertia_node_;         ///< The scene node the collision meshes are attached to
  rviz_rendering::Shape * mass_shape_;     ///< The shape representing the mass
  rviz_rendering::Shape * inertia_shape_;  ///< The shape representing the inertia

  Ogre::RibbonTrail * trail_;

  float material_alpha_;  ///< If material is not a texture, this saves the alpha value set
///< in the URDF, otherwise is 1.0.
  float robot_alpha_;  ///< Alpha value from top-level robot alpha Property
///< (set via setRobotAlpha()).

  bool only_render_depth_;
  bool is_selectable_;

  RobotLinkSelectionHandlerPtr selection_handler_;

  Ogre::MaterialPtr color_material_;
  bool using_color_;

  std::string error;

  friend class RobotLinkSelectionHandler;
};

}  // namespace robot
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__ROBOT__ROBOT_LINK_HPP_
