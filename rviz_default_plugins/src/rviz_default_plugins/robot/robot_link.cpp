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

#include "rviz_default_plugins/robot/robot_link.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreRibbonTrail.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>
#include <OgreTextureManager.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>

#include <QFileInfo>  // NOLINT cpplint cannot handle include order here

#include <ignition/math/Inertial.hh>
#include <ignition/math/MassMatrix3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "resource_retriever/retriever.hpp"

#include "rviz_default_plugins/robot/robot_joint.hpp"
#include "rviz_default_plugins/robot/robot.hpp"

#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/mesh_loader.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/object.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/interaction/selection_manager.hpp"

#define RVIZ_RESOURCE_GROUP "rviz_rendering"

using rviz_rendering::Axes;
using rviz_rendering::Shape;

namespace rviz_default_plugins
{
namespace robot
{

using rviz_common::properties::Property;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::QuaternionProperty;
using rviz_common::properties::VectorProperty;

class RobotLinkSelectionHandler : public rviz_common::interaction::SelectionHandler
{
public:
  RobotLinkSelectionHandler(RobotLink * link, rviz_common::DisplayContext * context);
  ~RobotLinkSelectionHandler() override;

  void createProperties(
    const rviz_common::interaction::Picked & obj,
    Property * parent_property) override;
  void updateProperties() override;

  void preRenderPass(uint32_t pass) override;
  void postRenderPass(uint32_t pass) override;

private:
  RobotLink * link_;
  rviz_common::properties::VectorProperty * position_property_;
  rviz_common::properties::QuaternionProperty * orientation_property_;

  template<typename T, typename ... Args>
  friend typename std::shared_ptr<T> rviz_common::interaction::createSelectionHandler(
    Args ... arguments);
};

RobotLinkSelectionHandler::RobotLinkSelectionHandler(
  RobotLink * link,
  rviz_common::DisplayContext * context)
: SelectionHandler(context),
  link_(link),
  position_property_(nullptr),
  orientation_property_(nullptr)
{}

RobotLinkSelectionHandler::~RobotLinkSelectionHandler() = default;

void RobotLinkSelectionHandler::createProperties(
  const rviz_common::interaction::Picked & obj,
  rviz_common::properties::Property * parent_property)
{
  (void) obj;
  Property * group = new Property(
    "Link " + QString::fromStdString(link_->getName()), QVariant(), "", parent_property);
  properties_.push_back(group);

  position_property_ = new VectorProperty("Position", Ogre::Vector3::ZERO, "", group);
  position_property_->setReadOnly(true);

  orientation_property_ = new QuaternionProperty(
    "Orientation", Ogre::Quaternion::IDENTITY, "", group);
  orientation_property_->setReadOnly(true);

  group->expand();
}

void RobotLinkSelectionHandler::updateProperties()
{
  position_property_->setVector(link_->getPosition());
  orientation_property_->setQuaternion(link_->getOrientation());
}


void RobotLinkSelectionHandler::preRenderPass(uint32_t pass)
{
  (void) pass;
  if (!link_->is_selectable_) {
    if (link_->visual_node_) {
      link_->visual_node_->setVisible(false);
    }
    if (link_->collision_node_) {
      link_->collision_node_->setVisible(false);
    }
    if (link_->mass_node_) {
      link_->mass_node_->setVisible(false);
    }
    if (link_->inertia_node_) {
      link_->inertia_node_->setVisible(false);
    }
    if (link_->trail_) {
      link_->trail_->setVisible(false);
    }
    if (link_->axes_) {
      link_->axes_->getSceneNode()->setVisible(false);
    }
  }
}

void RobotLinkSelectionHandler::postRenderPass(uint32_t pass)
{
  (void) pass;
  if (!link_->is_selectable_) {
    link_->updateVisibility();
  }
}

RobotLink::RobotLink(
  Robot * robot,
  const urdf::LinkConstSharedPtr & link,
  const std::string & parent_joint_name,
  bool visual,
  bool collision,
  bool mass,
  bool inertia)
: RobotElementBaseClass(robot, link->name),
  scene_manager_(robot->getDisplayContext()->getSceneManager()),
  context_(robot->getDisplayContext()),
  parent_joint_name_(parent_joint_name),
  visual_node_(nullptr),
  collision_node_(nullptr),
  mass_node_(nullptr),
  inertia_node_(nullptr),
  mass_shape_(nullptr),
  inertia_shape_(nullptr),
  trail_(nullptr),
  material_alpha_(1.0),
  robot_alpha_(1.0),
  only_render_depth_(false),
  is_selectable_(true),
  using_color_(false)
{
  setProperties(link);

  visual_node_ = robot_->getVisualNode()->createChildSceneNode();
  collision_node_ = robot_->getCollisionNode()->createChildSceneNode();
  mass_node_ = robot_->getOtherNode()->createChildSceneNode();
  inertia_node_ = robot_->getOtherNode()->createChildSceneNode();

  // create material for coloring links
  static int count = 1;
  std::string color_material_name = "robot link color material " + std::to_string(count++);

  color_material_ =
    rviz_rendering::MaterialManager::createMaterialWithLighting(color_material_name);

  // create the ogre objects to display
  if (visual) {
    createVisual(link);
  }

  if (collision) {
    createCollision(link);
  }

  if (mass) {
    createMass(link);
  }

  if (inertia) {
    createInertia(link);
  }

  if (collision || visual) {
    createSelection();
  }

  createDescription(link);

  if (!hasGeometry()) {
    robot_element_property_->setIcon(
      rviz_common::loadPixmap(
        "package://rviz_default_plugins/icons/classes/RobotLinkNoGeom.png"));
    alpha_property_->hide();
    robot_element_property_->setValue(QVariant());
  }
}

void RobotLink::setProperties(const urdf::LinkConstSharedPtr & link)
{
  robot_element_property_ = new Property(
    link->name.c_str(), true, "", nullptr, SLOT(updateVisibility()), this);
  robot_element_property_->setIcon(
    rviz_common::loadPixmap(
      "package://rviz_default_plugins/icons/classes/RobotLink.png"));

  details_ = new Property("Details", QVariant(), "", nullptr);

  alpha_property_ = new FloatProperty(
    "Alpha", 1,
    "Amount of transparency to apply to this link.",
    robot_element_property_, SLOT(updateAlpha()), this);

  trail_property_ = new Property(
    "Show Trail", false,
    "Enable/disable a 2 meter \"ribbon\" which follows this link.",
    robot_element_property_, SLOT(updateTrail()), this);

  axes_property_ = new Property(
    "Show Axes", false,
    "Enable/disable showing the axes of this link.",
    robot_element_property_, SLOT(updateAxes()), this);

  position_property_ = new VectorProperty(
    "Position", Ogre::Vector3::ZERO,
    "Position of this link, in the current Fixed Frame.  (Not editable)",
    robot_element_property_);
  position_property_->setReadOnly(true);

  orientation_property_ = new QuaternionProperty(
    "Orientation", Ogre::Quaternion::IDENTITY,
    "Orientation of this link, in the current Fixed Frame.  (Not editable)",
    robot_element_property_);
  orientation_property_->setReadOnly(true);

  robot_element_property_->collapse();
}

void RobotLink::createDescription(const urdf::LinkConstSharedPtr & link)
{
  std::stringstream desc;
  if (parent_joint_name_.empty()) {
    desc << "Root Link <b>" << name_ << "</b>";
  } else {
    desc << "Link <b>" << name_ << "</b>";
    desc << " with parent joint <b>" << parent_joint_name_ << "</b>";
  }

  if (link->child_joints.empty()) {
    desc << " has no children.";
  } else {
    desc << " has " << link->child_joints.size();

    if (link->child_joints.size() > 1) {
      desc << " child joints: ";
    } else {
      desc << " child joint: ";
    }

    auto child_it = link->child_joints.begin();
    auto child_end = link->child_joints.end();
    for (; child_it != child_end; ++child_it) {
      urdf::Joint * child_joint = child_it->get();
      if (child_joint && !child_joint->name.empty()) {
        child_joint_names_.push_back(child_joint->name);
        desc << "<b>" << child_joint->name << "</b>" << ((child_it + 1 == child_end) ? "." : ", ");
      }
    }
  }
  if (hasGeometry()) {
    desc << "  Check/uncheck to show/hide this link in the display.";
    if (visual_meshes_.empty()) {
      desc << "  This link has collision geometry but no visible geometry.";
    } else if (collision_meshes_.empty()) {
      desc << "  This link has visible geometry but no collision geometry.";
    }
  } else {
    desc << "  This link has NO geometry.";
  }

  robot_element_property_->setDescription(desc.str().c_str());
}

RobotLink::~RobotLink()
{
  for (auto & visual_mesh : visual_meshes_) {
    scene_manager_->destroyEntity(visual_mesh);
  }

  for (auto & collision_mesh : collision_meshes_) {
    scene_manager_->destroyEntity(collision_mesh);
  }

  scene_manager_->destroySceneNode(visual_node_);
  scene_manager_->destroySceneNode(collision_node_);
  scene_manager_->destroySceneNode(mass_node_);
  scene_manager_->destroySceneNode(inertia_node_);

  if (trail_) {
    scene_manager_->destroyRibbonTrail(trail_);
  }

  delete details_;
  delete robot_element_property_;
}

void RobotLink::addError(const char * format, ...)
{
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (!error.empty()) {
    error.append("\n");
  }
  error.append(buffer);
}

const std::string RobotLink::getGeometryErrors() const
{
  return error;
}

void RobotLink::setRobotAlpha(float a)
{
  robot_alpha_ = a;
  updateAlpha();
}

void RobotLink::setTransforms(
  const Ogre::Vector3 & visual_position, const Ogre::Quaternion & visual_orientation,
  const Ogre::Vector3 & collision_position, const Ogre::Quaternion & collision_orientation)
{
  if (visual_node_) {
    visual_node_->setPosition(visual_position);
    visual_node_->setOrientation(visual_orientation);
  }

  if (collision_node_) {
    collision_node_->setPosition(collision_position);
    collision_node_->setOrientation(collision_orientation);
  }

  if (mass_node_) {
    mass_node_->setPosition(visual_position);
    mass_node_->setOrientation(visual_orientation);
  }

  if (inertia_node_) {
    inertia_node_->setPosition(visual_position);
    inertia_node_->setOrientation(visual_orientation);
  }

  position_property_->setVector(visual_position);
  orientation_property_->setQuaternion(visual_orientation);

  if (axes_) {
    axes_->setPosition(visual_position);
    axes_->setOrientation(visual_orientation);
  }
}

void RobotLink::setToErrorMaterial()
{
  for (auto & visual_mesh : visual_meshes_) {
    visual_mesh->setMaterialName("BaseWhiteNoLighting");
  }
  for (auto & collision_mesh : collision_meshes_) {
    collision_mesh->setMaterialName("BaseWhiteNoLighting");
  }
}

void RobotLink::setToNormalMaterial()
{
  if (using_color_) {
    for (auto & visual_mesh : visual_meshes_) {
      visual_mesh->setMaterial(color_material_);
    }
    for (auto & collision_mesh : collision_meshes_) {
      collision_mesh->setMaterial(color_material_);
    }
  } else {
    for (const auto & material_entry : materials_) {
      material_entry.first->setMaterial(material_entry.second);
    }
  }
}

void RobotLink::setColor(float red, float green, float blue)
{
  Ogre::ColourValue color = color_material_->getTechnique(0)->getPass(0)->getDiffuse();
  color.r = red;
  color.g = green;
  color.b = blue;
  color_material_->getTechnique(0)->setAmbient(0.5f * color);
  color_material_->getTechnique(0)->setDiffuse(color);

  using_color_ = true;
  setToNormalMaterial();
}

void RobotLink::unsetColor()
{
  using_color_ = false;
  setToNormalMaterial();
}

bool RobotLink::setSelectable(bool selectable)
{
  bool old = is_selectable_;
  is_selectable_ = selectable;
  return old;
}

bool RobotLink::getSelectable()
{
  return is_selectable_;
}

bool RobotLink::hasGeometry() const
{
  return visual_meshes_.size() + collision_meshes_.size() > 0;
}

void RobotLink::setOnlyRenderDepth(bool onlyRenderDepth)
{
  setRenderQueueGroup(onlyRenderDepth ? Ogre::RENDER_QUEUE_BACKGROUND : Ogre::RENDER_QUEUE_MAIN);
  only_render_depth_ = onlyRenderDepth;
  updateAlpha();
}

void RobotLink::updateVisibility()
{
  bool enabled = getEnabled();

  robot_->calculateJointCheckboxes();

  if (visual_node_) {
    visual_node_->setVisible(enabled && robot_->isVisible() && robot_->isVisualVisible());
  }
  if (collision_node_) {
    collision_node_->setVisible(enabled && robot_->isVisible() && robot_->isCollisionVisible());
  }
  if (mass_node_) {
    mass_node_->setVisible(enabled && robot_->isVisible() && robot_->isMassVisible());
  }
  if (inertia_node_) {
    inertia_node_->setVisible(enabled && robot_->isVisible() && robot_->isInertiaVisible());
  }
  if (trail_) {
    trail_->setVisible(enabled && robot_->isVisible());
  }
  if (axes_) {
    axes_->getSceneNode()->setVisible(enabled && robot_->isVisible());
  }
}

void RobotLink::updateAlpha()
{
  float link_alpha = alpha_property_->getFloat();
  for (const auto & material_entry : materials_) {
    const Ogre::MaterialPtr & material = material_entry.second;

    if (only_render_depth_) {
      material->setColourWriteEnabled(false);
      material->setDepthWriteEnabled(true);
    } else {
      Ogre::ColourValue color = material->getTechnique(0)->getPass(0)->getDiffuse();
      color.a = robot_alpha_ * material_alpha_ * link_alpha;
      material->setDiffuse(color);

      rviz_rendering::MaterialManager::enableAlphaBlending(material, color.a);
    }
  }

  Ogre::ColourValue color = color_material_->getTechnique(0)->getPass(0)->getDiffuse();
  color.a = robot_alpha_ * link_alpha;
  color_material_->setDiffuse(color);

  rviz_rendering::MaterialManager::enableAlphaBlending(color_material_, color.a);
}

void RobotLink::updateTrail()
{
  if (trail_property_->getValue().toBool()) {
    if (!trail_) {
      if (visual_node_) {
        static int count = 0;
        std::string link_name = "Trail for link " + name_ + std::to_string(count++);
        trail_ = scene_manager_->createRibbonTrail(link_name);
        trail_->setMaxChainElements(100);
        trail_->setInitialWidth(0, 0.01f);
        trail_->setInitialColour(0, 0.0f, 0.5f, 0.5f);
        trail_->addNode(visual_node_);
        trail_->setTrailLength(2.0f);
        trail_->setVisible(getEnabled());
        robot_->getOtherNode()->attachObject(trail_);
      } else {
        RVIZ_COMMON_LOG_ERROR_STREAM(
          "No visual node for link '" << name_ << "', cannot create a trail");
      }
    }
  } else {
    if (trail_) {
      scene_manager_->destroyRibbonTrail(trail_);
      trail_ = nullptr;
    }
  }
}

void RobotLink::setRenderQueueGroup(Ogre::uint8 group)
{
  for (auto child_node : visual_node_->getChildren()) {
    auto child = dynamic_cast<Ogre::SceneNode *>(child_node);
    if (child) {
      auto attached_objects = child->getAttachedObjects();
      for (const auto & object : attached_objects) {
        object->setRenderQueueGroup(group);
      }
    }
  }
}

bool RobotLink::getEnabled() const
{
  if (!hasGeometry()) {
    return true;
  }
  return robot_element_property_->getValue().toBool();
}

Ogre::Entity * RobotLink::createEntityForGeometryElement(
  const urdf::LinkConstSharedPtr & link,
  const urdf::Geometry & geom,
  const urdf::Pose & origin,
  const std::string material_name,
  Ogre::SceneNode * scene_node)
{
  Ogre::Entity * entity = nullptr;  // default in case nothing works.
  Ogre::SceneNode * offset_node = scene_node->createChildSceneNode();

  static int count = 0;
  std::string entity_name = "Robot Link" + std::to_string(count++);

  Ogre::Vector3 scale(Ogre::Vector3::UNIT_SCALE);

  Ogre::Vector3 offset_position(
    static_cast<float>(origin.position.x),
    static_cast<float>(origin.position.y),
    static_cast<float>(origin.position.z));

  Ogre::Quaternion offset_orientation = Ogre::Quaternion(
    static_cast<float>(origin.rotation.w),
    static_cast<float>(origin.rotation.x),
    static_cast<float>(origin.rotation.y),
    static_cast<float>(origin.rotation.z));

  switch (geom.type) {
    case urdf::Geometry::SPHERE:
      {
        auto sphere = dynamic_cast<const urdf::Sphere &>(geom);
        entity = Shape::createEntity(entity_name, Shape::Sphere, scene_manager_);

        scale = Ogre::Vector3(
          static_cast<float>(sphere.radius * 2),
          static_cast<float>(sphere.radius * 2),
          static_cast<float>(sphere.radius * 2));
        break;
      }
    case urdf::Geometry::BOX:
      {
        auto box = dynamic_cast<const urdf::Box &>(geom);
        entity = Shape::createEntity(entity_name, Shape::Cube, scene_manager_);

        scale = Ogre::Vector3(
          static_cast<float>(box.dim.x),
          static_cast<float>(box.dim.y),
          static_cast<float>(box.dim.z));
        break;
      }
    case urdf::Geometry::CYLINDER:
      {
        auto cylinder = dynamic_cast<const urdf::Cylinder &>(geom);

        Ogre::Quaternion rotX;
        rotX.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
        offset_orientation = offset_orientation * rotX;

        entity = Shape::createEntity(entity_name, Shape::Cylinder, scene_manager_);
        scale = Ogre::Vector3(
          static_cast<float>(cylinder.radius * 2),
          static_cast<float>(cylinder.length),
          static_cast<float>(cylinder.radius * 2));
        break;
      }
    case urdf::Geometry::MESH:
      {
        auto mesh = dynamic_cast<const urdf::Mesh &>(geom);

        if (mesh.filename.empty()) {
          return nullptr;
        }

        scale = Ogre::Vector3(
          static_cast<float>(mesh.scale.x),
          static_cast<float>(mesh.scale.y),
          static_cast<float>(mesh.scale.z));

        const std::string & model_name = mesh.filename;

        try {
          if (rviz_rendering::loadMeshFromResource(model_name) == nullptr) {
            addError("Could not load mesh resource '%s'", model_name.c_str());
          } else {
            entity = scene_manager_->createEntity(entity_name, model_name);
          }
        } catch (Ogre::InvalidParametersException & e) {
          RVIZ_COMMON_LOG_ERROR_STREAM(
            "Could not convert mesh resource '" << model_name << "' for link '" << link->name <<
              "'. It may be an empty mesh: " << e.what());
          addError("Could not convert mesh resource '%s': %s", model_name.c_str(), e.what());
        } catch (Ogre::Exception & e) {
          RVIZ_COMMON_LOG_ERROR_STREAM(
            "could not load model '" << model_name << "' for link '" << link->name + "': " <<
              e.what());
          addError("Could not load model '%s': %s", model_name.c_str(), e.what());
        }
        break;
      }
    default:
      RVIZ_COMMON_LOG_ERROR_STREAM("Unsupported geometry type for element: " << geom.type);
      break;
  }

  if (entity) {
    offset_node->attachObject(entity);
    offset_node->setScale(scale);
    offset_node->setPosition(offset_position);
    offset_node->setOrientation(offset_orientation);

    assignMaterialsToEntities(link, material_name, entity);
  }
  return entity;
}

void RobotLink::assignMaterialsToEntities(
  const urdf::LinkConstSharedPtr & link,
  const std::string & material_name,
  const Ogre::Entity * entity)
{
  static int material_count = 0;
  if (default_material_name_.empty()) {
    default_material_ = getMaterialForLink(link);

    std::string cloned_name =
      default_material_->getName() + "_" + std::to_string(material_count++) + "Robot";

    default_material_ = default_material_->clone(cloned_name);
    default_material_name_ = default_material_->getName();
  }

  for (uint32_t i = 0; i < entity->getNumSubEntities(); ++i) {
    default_material_ = getMaterialForLink(link, material_name);
    std::string cloned_name =
      default_material_->getName() + "_" + std::to_string(material_count++) + "Robot";

    default_material_ = default_material_->clone(cloned_name);
    default_material_name_ = default_material_->getName();

    // Assign materials only if the submesh does not have one already

    Ogre::SubEntity * sub = entity->getSubEntity(i);
    const std::string & sub_material_name = sub->getMaterialName();

    if (sub_material_name == "BaseWhite" || sub_material_name == "BaseWhiteNoLighting") {
      sub->setMaterialName(default_material_name_);
    } else {
      // Need to clone here due to how selection works.
      // Once selection id is done per object and not per material,
      // this can go away
      std::string sub_cloned_name =
        sub_material_name + "_" + std::to_string(material_count++) + "Robot";
      sub->getMaterial()->clone(sub_cloned_name);
      sub->setMaterialName(sub_cloned_name);
    }

    materials_[sub] = sub->getMaterial();
  }
}

Ogre::MaterialPtr RobotLink::getMaterialForLink(
  const urdf::LinkConstSharedPtr & link, const std::string material_name)
{
  if (!link->visual || !link->visual->material) {
    return Ogre::MaterialManager::getSingleton().getByName("RVIZ/ShadedRed");
  }

  static int count = 0;
  std::string link_material_name = "Robot Link Material" + std::to_string(count++);

  auto material_for_link =
    rviz_rendering::MaterialManager::createMaterialWithShadowsAndLighting(link_material_name);

  urdf::VisualSharedPtr visual = getVisualWithMaterial(link, material_name);

  if (visual->material->texture_filename.empty()) {
    const urdf::Color & color = visual->material->color;
    material_for_link->getTechnique(0)->setAmbient(color.r * 0.5f, color.g * 0.5f, color.b * 0.5f);
    material_for_link->getTechnique(0)->setDiffuse(color.r, color.g, color.b, color.a);

    material_alpha_ = color.a;
  } else {
    loadMaterialFromTexture(material_for_link, visual);
  }

  return material_for_link;
}

urdf::VisualSharedPtr RobotLink::getVisualWithMaterial(
  const urdf::LinkConstSharedPtr & link, const std::string & material_name) const
{
  urdf::VisualSharedPtr visual = link->visual;
  for (const auto & visual_array_element : link->visual_array) {
    if (visual_array_element &&
      !material_name.empty() &&
      visual_array_element->material_name == material_name)
    {
      visual = visual_array_element;
      break;
    }
  }
  return visual;
}

void RobotLink::loadMaterialFromTexture(
  Ogre::MaterialPtr & material_for_link, const urdf::VisualSharedPtr & visual) const
{
  std::string filename = visual->material->texture_filename;
  if (!Ogre::TextureManager::getSingleton().resourceExists(filename, RVIZ_RESOURCE_GROUP)) {
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res;
    try {
      res = retriever.get(filename);
    } catch (resource_retriever::Exception & e) {
      RVIZ_COMMON_LOG_ERROR(e.what());
    }

    if (res.size != 0) {
      Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
      Ogre::Image image;
      std::string extension =
        QFileInfo(QString::fromStdString(filename)).completeSuffix().toStdString();

      if (extension[0] == '.') {
        extension = extension.substr(1, extension.size() - 1);
      }

      try {
        image.load(stream, extension);
        Ogre::TextureManager::getSingleton().loadImage(
          filename,
          RVIZ_RESOURCE_GROUP,
          image);
      } catch (Ogre::Exception & e) {
        RVIZ_COMMON_LOG_ERROR_STREAM("Could not load texture [" << filename << "]: " << e.what());
      }
    }
  }

  Ogre::Pass * pass = material_for_link->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState * tex_unit = pass->createTextureUnitState();
  tex_unit->setTextureName(filename);
}

void RobotLink::createCollision(const urdf::LinkConstSharedPtr & link)
{
  createVisualizable<urdf::CollisionSharedPtr>(
    link, collision_meshes_, link->collision_array, link->collision, collision_node_);

  collision_node_->setVisible(getEnabled());
}

void RobotLink::createVisual(const urdf::LinkConstSharedPtr & link)
{
  createVisualizable<urdf::VisualSharedPtr>(
    link, visual_meshes_, link->visual_array, link->visual, visual_node_);

  visual_node_->setVisible(getEnabled());
}

void RobotLink::createMass(const urdf::LinkConstSharedPtr & link)
{
  if (link->inertial) {
    // display a sphere sized as if it were a ball of lead
    // with the same mass as the link
    // mass = 4/3 pi r^3 density
    urdf::Pose pose = link->inertial->origin;
    Ogre::Vector3 translate(pose.position.x, pose.position.y, pose.position.z);
    Ogre::SceneNode * offset_node = mass_node_->createChildSceneNode(translate);
    mass_shape_ = new Shape(Shape::Sphere, scene_manager_, offset_node);

    double density_of_lead = 11340;
    double diameter = 2 * cbrt((0.75 * link->inertial->mass) / (M_PI * density_of_lead));
    mass_shape_->setColor(1, 0, 0, 1);
    mass_shape_->setScale(Ogre::Vector3(diameter, diameter, diameter));
  }
}

void RobotLink::createInertia(const urdf::LinkConstSharedPtr & link)
{
  if (link->inertial) {
    const ignition::math::Vector3d i_xx_yy_zz(
      link->inertial->ixx,
      link->inertial->iyy,
      link->inertial->izz);
    const ignition::math::Vector3d Ixyxzyz(
      link->inertial->ixy,
      link->inertial->ixz,
      link->inertial->iyz);
    ignition::math::MassMatrix3d mass_matrix(link->inertial->mass, i_xx_yy_zz, Ixyxzyz);

    ignition::math::Vector3d box_scale;
    ignition::math::Quaterniond box_rot;
    if (!mass_matrix.EquivalentBox(box_scale, box_rot)) {
      // Invalid inertia, load with default scale
      if (link->parent_joint && link->parent_joint->type != urdf::Joint::FIXED) {
        // Do not show error message for base link or static links
        RVIZ_COMMON_LOG_ERROR_STREAM(
          "The link " << link->name << " has unrealistic "
            "inertia, so the equivalent inertia box will not be shown.\n");
      }
      return;
    }
    Ogre::Vector3 translate(
      link->inertial->origin.position.x,
      link->inertial->origin.position.y,
      link->inertial->origin.position.z);
    Ogre::Quaternion rotate(box_rot.W(), box_rot.X(), box_rot.Y(), box_rot.Z());
    Ogre::SceneNode * offset_node = inertia_node_->createChildSceneNode(translate, rotate);
    inertia_shape_ = new Shape(Shape::Cube, scene_manager_, offset_node);

    inertia_shape_->setColor(1, 0, 0, 1);
    inertia_shape_->setScale(Ogre::Vector3(box_scale.X(), box_scale.Y(), box_scale.Z()));
  }
}

void RobotLink::createSelection()
{
  selection_handler_ = rviz_common::interaction::createSelectionHandler<RobotLinkSelectionHandler>(
    this, context_);
  for (auto & visual_mesh : visual_meshes_) {
    selection_handler_->addTrackedObject(visual_mesh);
  }
  for (auto & collision_mesh : collision_meshes_) {
    selection_handler_->addTrackedObject(collision_mesh);
  }
}

}  // namespace robot
}  // namespace rviz_default_plugins
