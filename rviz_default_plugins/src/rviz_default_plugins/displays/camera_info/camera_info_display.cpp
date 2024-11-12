//
// Copyright (c) 2024, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "rviz_default_plugins/displays/camera_info/camera_info_display.hpp"

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreHardwarePixelBuffer.h>

#include <QString>  // NOLINT: cpplint is unable to handle the include order here

#include <rviz_common/uniform_string_stream.hpp>
#include <rviz_common/properties/parse_color.hpp>

#include <rviz_rendering/objects/triangle_polygon.hpp>

namespace rviz_default_plugins
{
namespace displays
{
static const char * const CAM_INFO_STATUS = "Camera Info";

CameraInfoDisplay::CameraInfoDisplay()
: rviz_common::MessageFilterDisplay<sensor_msgs::msg::CameraInfo>()
{
  far_clip_distance_property_ = new rviz_common::properties::FloatProperty(
    "Far clip",
    1.0,
    "Far clip distance from the origin of camera info",
    this, SLOT(updateFarClipDistance()));
  show_edges_property_ = new rviz_common::properties::BoolProperty(
    "Show edges",
    true,
    "Show edges of the region of the camera info",
    this, SLOT(updateShowEdges()));
  show_polygons_property_ = new rviz_common::properties::BoolProperty(
    "Show polygons",
    true,
    "Show polygons of the region of the camera info",
    this, SLOT(updateShowPolygons()));
  not_show_side_polygons_property_ = new rviz_common::properties::BoolProperty(
    "Not show side polygons",
    true,
    "Do not show polygons of the region of the camera info",
    this, SLOT(updateNotShowSidePolygons()));

  color_property_ = new rviz_common::properties::ColorProperty(
    "Color",
    QColor(85, 255, 255),
    "Color of CameraInfo",
    this, SLOT(updateColor()));
  edge_color_property_ = new rviz_common::properties::ColorProperty(
    "Edge color",
    QColor(125, 125, 125),
    "Edge color of CameraInfo",
    this, SLOT(updateEdgeColor()));
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha",
    0.5,
    "Alpha blending value",
    this, SLOT(updateAlpha()));
}

CameraInfoDisplay::~CameraInfoDisplay()
{
  if (edges_) {
    edges_->clear();
  }
  polygons_.clear();
  delete far_clip_distance_property_;
  delete color_property_;
  delete alpha_property_;
  delete show_polygons_property_;
  delete edge_color_property_;
}

void CameraInfoDisplay::reset()
{
  MFDClass::reset();
  if (edges_) {
    edges_->clear();
  }
  polygons_.clear();
  camera_info_ = sensor_msgs::msg::CameraInfo::ConstSharedPtr();
}

void CameraInfoDisplay::onInitialize()
{
  MFDClass::onInitialize();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  updateColor();
  updateAlpha();
  updateFarClipDistance();
  updateShowPolygons();
  updateNotShowSidePolygons();
  updateShowEdges();
  updateEdgeColor();
}

void CameraInfoDisplay::processMessage(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  if (!isSameCameraInfo(msg)) {
    createCameraInfoShapes(msg);
  }
  // move scene_node according to tf
  Ogre::Vector3 position;
  Ogre::Quaternion quaternion;
  std::string frame_id = msg->header.frame_id;
  if (frame_id[0] == '/') {
    frame_id = frame_id.substr(1, frame_id.size());
  }
  if (!context_->getFrameManager()->getTransform(
      frame_id,
      msg->header.stamp,
      position,
      quaternion))
  {
    std::ostringstream sstm;
    sstm << "Error transforming pose '" << qPrintable(getName()) << "' from frame '"
         << msg->header.frame_id.c_str() << "' to frame '" << qPrintable(fixed_frame_) << "'";
    setStatus(
      rviz_common::properties::StatusLevel::Warn, CAM_INFO_STATUS,
      QString(sstm.str().c_str()));
  }
  scene_node_->setPosition(position);
  scene_node_->setOrientation(quaternion);
  camera_info_ = msg;
}

void CameraInfoDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
}

bool CameraInfoDisplay::isSameCameraInfo(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (camera_info_) {
    bool meta_same_p =
      msg->header.frame_id == camera_info_->header.frame_id &&
      msg->height == camera_info_->height &&
      msg->width == camera_info_->width &&
      msg->distortion_model == camera_info_->distortion_model &&
      msg->roi.x_offset == camera_info_->roi.x_offset &&
      msg->roi.y_offset == camera_info_->roi.y_offset &&
      msg->roi.height == camera_info_->roi.height &&
      msg->roi.width == camera_info_->roi.width;
    if (meta_same_p) {
      for (size_t i = 0; i < msg->p.size(); i++) {
        if (msg->p[i] != camera_info_->p[i]) {
          return false;
        }
      }
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

void CameraInfoDisplay::addPointToEdge(
  const Ogre::Vector3 & point)
{
  edges_->addPoint(point);
}

void CameraInfoDisplay::addPolygon(
  const Ogre::Vector3 & O, const Ogre::Vector3 & A, const Ogre::Vector3 & B,
  std::string name, bool use_color, bool upper_triangle)
{
  Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_);
  color.a = alpha_;
  std::shared_ptr<rviz_rendering::TrianglePolygon> triangle =
    std::make_shared<rviz_rendering::TrianglePolygon>(
    scene_manager_,
    scene_node_,
    O, A, B, name,
    color,
    use_color,
    upper_triangle);
  polygons_.push_back(triangle);
}

void CameraInfoDisplay::prepareMaterial()
{
  if (texture_ == nullptr) {
    // material
    static uint32_t count = 0;
    rviz_common::UniformStringStream ss;
    ss << "CameraInfoDisplayPolygon" << count++;
    material_ =
      Ogre::MaterialManager::getSingletonPtr()->create(
      ss.str(),
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    texture_ = Ogre::TextureManager::getSingletonPtr()->createManual(
      material_->getName() + "Texture",          // name
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D, 1, 1, 0, Ogre::PF_A8R8G8B8, Ogre::TU_DEFAULT);
    material_->getTechnique(0)->getPass(0)->setColourWriteEnabled(true);
    Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_);
    color.a = alpha_;
    material_->getTechnique(0)->getPass(0)->setAmbient(color);
    material_->setReceiveShadows(false);
    material_->getTechnique(0)->setLightingEnabled(true);
    material_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
    material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    material_->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);

    material_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_->getName());
    material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  }
}

Ogre::Vector3 projectPixelTo3dRay(
  double x, double y, double fx, double fy, double cx, double cy,
  double Tx, double Ty)
{
  Ogre::Vector3 ray;
  ray.x = (x - cx - Tx) / fx;
  ray.y = (y - cy - Ty) / fy;
  ray.z = 1.0;
  return ray;
}

void CameraInfoDisplay::createCameraInfoShapes(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  polygons_.clear();
  if (edges_) {
    edges_->clear();
  }
  // fx and fy should not be equal 0.
  if (msg->p[0] == 0.0 || msg->p[5] == 0.0) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Camera Info",
      "Invalid intrinsic matrix");
    return;
  }
  setStatus(rviz_common::properties::StatusProperty::Ok, "Camera Info", "OK");

  // initialize BillboardLine
  if (!edges_) {
    edges_.reset(
      new rviz_rendering::BillboardLine(
        context_->getSceneManager(),
        scene_node_));
    edges_->setLineWidth(0.01f);
  }

  int height = msg->roi.height ? msg->roi.height : msg->height;
  int width = msg->roi.width ? msg->roi.width : msg->width;
  if (msg->binning_y > 0) {
    height /= msg->binning_y;
  }
  if (msg->binning_x > 0) {
    width /= msg->binning_x;
  }

  Ogre::Vector2 a(0, 0), b(width, 0),
  c(width, height), d(0, height);
  // all the z = 1.0
  Ogre::Vector3 A = projectPixelTo3dRay(
    a.x, a.y, msg->p[0], msg->p[5], msg->p[2], msg->p[6],
    msg->p[3], msg->p[7]);
  Ogre::Vector3 B = projectPixelTo3dRay(
    b.x, b.y, msg->p[0], msg->p[5], msg->p[2], msg->p[6],
    msg->p[3], msg->p[7]);
  Ogre::Vector3 C = projectPixelTo3dRay(
    c.x, c.y, msg->p[0], msg->p[5], msg->p[2], msg->p[6],
    msg->p[3], msg->p[7]);
  Ogre::Vector3 D = projectPixelTo3dRay(
    d.x, d.y, msg->p[0], msg->p[5], msg->p[2], msg->p[6],
    msg->p[3], msg->p[7]);

  Ogre::Vector3 scaled_A = A * far_clip_distance_;
  Ogre::Vector3 scaled_B = B * far_clip_distance_;
  Ogre::Vector3 scaled_C = C * far_clip_distance_;
  Ogre::Vector3 scaled_D = D * far_clip_distance_;

  Ogre::Vector3 O(0, 0, 0);

  // build polygons
  if (show_polygons_) {
    Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_);
    color.a = alpha_;
    prepareMaterial();
    if (!not_show_side_polygons_) {
      material_->getTechnique(0)->getPass(0)->setAmbient(color);
      {
        texture_->getBuffer()->lock(Ogre::HardwareBuffer::HBL_NORMAL);
        const Ogre::PixelBox & pixelBox =
          texture_->getBuffer()->getCurrentLock();
        Ogre::uint8 * pDest = static_cast<Ogre::uint8 *>(pixelBox.data);
        memset(pDest, 0, 1);
        QImage Hud(pDest, 1, 1, QImage::Format_ARGB32);
        Hud.setPixel(0, 0, color_.rgba());
        texture_->getBuffer()->unlock();
      }
      addPolygon(O, scaled_B, scaled_A, material_->getName(), true, true);
      addPolygon(O, scaled_C, scaled_B, material_->getName(), true, true);
      addPolygon(O, scaled_D, scaled_C, material_->getName(), true, true);
      addPolygon(O, scaled_A, scaled_D, material_->getName(), true, true);
    }
  }
  if (show_edges_) {
    edges_->clear();
    edges_->setMaxPointsPerLine(2);
    edges_->setNumLines(8);
    edges_->setColor(
      edge_color_.red() / 255.0,
      edge_color_.green() / 255.0,
      edge_color_.blue() / 255.0,
      alpha_);
    addPointToEdge(O); addPointToEdge(scaled_A); edges_->finishLine();
    addPointToEdge(O); addPointToEdge(scaled_B); edges_->finishLine();
    addPointToEdge(O); addPointToEdge(scaled_C); edges_->finishLine();
    addPointToEdge(O); addPointToEdge(scaled_D); edges_->finishLine();
    addPointToEdge(scaled_A); addPointToEdge(scaled_B); edges_->finishLine();
    addPointToEdge(scaled_B); addPointToEdge(scaled_C); edges_->finishLine();
    addPointToEdge(scaled_C); addPointToEdge(scaled_D); edges_->finishLine();
    addPointToEdge(scaled_D); addPointToEdge(scaled_A);
  }
}

void CameraInfoDisplay::updateColor()
{
  color_ = color_property_->getColor();
  if (camera_info_) {
    createCameraInfoShapes(camera_info_);
  }
}

void CameraInfoDisplay::updateEdgeColor()
{
  edge_color_ = edge_color_property_->getColor();
  if (camera_info_) {
    createCameraInfoShapes(camera_info_);
  }
}

void CameraInfoDisplay::updateAlpha()
{
  alpha_ = alpha_property_->getFloat();
  if (camera_info_) {
    createCameraInfoShapes(camera_info_);
  }
}

void CameraInfoDisplay::updateFarClipDistance()
{
  far_clip_distance_ = far_clip_distance_property_->getFloat();
  if (camera_info_) {
    createCameraInfoShapes(camera_info_);
  }
}

void CameraInfoDisplay::updateShowPolygons()
{
  show_polygons_ = show_polygons_property_->getBool();
  if (show_polygons_) {
    not_show_side_polygons_property_->show();
  } else {
    not_show_side_polygons_property_->hide();
  }
  if (camera_info_) {
    createCameraInfoShapes(camera_info_);
  }
}

void CameraInfoDisplay::updateShowEdges()
{
  show_edges_ = show_edges_property_->getBool();
  if (camera_info_) {
    createCameraInfoShapes(camera_info_);
  }
}

void CameraInfoDisplay::updateNotShowSidePolygons()
{
  not_show_side_polygons_ = not_show_side_polygons_property_->getBool();
  if (camera_info_) {
    createCameraInfoShapes(camera_info_);
  }
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::CameraInfoDisplay, rviz_common::Display)
