/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <boost/bind.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>

#include "rviz_rendering/axes.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rviz_common/bit_allocator.hpp"
#include "rviz_common/frame_manager.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/display_group_visibility_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"

#include "camera_display.hpp"

namespace rviz_default_plugins
{

namespace displays
{

const QString CameraDisplay::BACKGROUND("background");
const QString CameraDisplay::OVERLAY("overlay");
const QString CameraDisplay::BOTH("background and overlay");

bool validateFloats(const sensor_msgs::msg::CameraInfo & msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.d);
  valid = valid && rviz_common::validateFloats(msg.k);
  valid = valid && rviz_common::validateFloats(msg.r);
  valid = valid && rviz_common::validateFloats(msg.p);
  return valid;
}

CameraDisplay::CameraDisplay()
  : texture_(), render_panel_(0), new_caminfo_(false),
  caminfo_ok_(false), force_render_(false)
{
  image_position_property_ = new rviz_common::properties::EnumProperty("Image Rendering", BOTH,
    "Render the image behind all other geometry or overlay it on top, or both.",
    this, SLOT(forceRender()));
  image_position_property_->addOption(BACKGROUND);
  image_position_property_->addOption(OVERLAY);
  image_position_property_->addOption(BOTH);

  alpha_property_ = new rviz_common::properties::FloatProperty("Overlay Alpha", 0.5,
    "The amount of transparency to apply to the camera image when rendered as overlay.",
    this, SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  zoom_property_ = new rviz_common::properties::FloatProperty("Zoom Factor", 1.0,
    "Set a zoom factor below 1 to see a larger part of the world, above 1 to magnify the image.",
    this, SLOT(forceRender()));
  zoom_property_->setMin(0.00001);
  zoom_property_->setMax(100000);
}

CameraDisplay::~CameraDisplay()
{
  if (initialized()) {
    // TODO(Martin-Idel-SI): Figure out whether this is still needed
//    render_panel_->getRenderWindow()->removeListener(this);

    unsubscribe();
//    caminfo_tf_filter_->clear();


    //workaround. delete results in a later crash
    render_panel_->hide();
    //delete render_panel_;

    delete bg_screen_rect_;
    delete fg_screen_rect_;

    bg_scene_node_->getParentSceneNode()->removeAndDestroyChild(bg_scene_node_->getName());
    fg_scene_node_->getParentSceneNode()->removeAndDestroyChild(fg_scene_node_->getName());

//    delete caminfo_tf_filter_;

    context_->visibilityBits()->freeBits(vis_bit_);
  }
}

void CameraDisplay::onInitialize()
{
  RTDClass::onInitialize();

//  caminfo_tf_filter_ = new tf::MessageFilter<sensor_msgs::CameraInfo>(*context_->getTFClient(),
//    fixed_frame_.toStdString(),
//    queue_size_property_->getInt(), update_nh_);

  bg_scene_node_ = scene_node_->createChildSceneNode();
  fg_scene_node_ = scene_node_->createChildSceneNode();

  {
    static int count = 0;
    rviz_common::UniformStringStream ss;
    ss << "CameraDisplayObject" << count++;

    //background rectangle
    bg_screen_rect_ = new Ogre::Rectangle2D(true);
    bg_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    ss << "Material";
    bg_material_ = Ogre::MaterialManager::getSingleton().create(ss.str(),
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    bg_material_->setDepthWriteEnabled(false);

    bg_material_->setReceiveShadows(false);
    bg_material_->setDepthCheckEnabled(false);

    bg_material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState * tu = bg_material_->getTechnique(0)->getPass(
      0)->createTextureUnitState();
    tu->setTextureName(texture_.getTexture()->getName());
    tu->setTextureFiltering(Ogre::TFO_NONE);
    tu->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0);

    bg_material_->setCullingMode(Ogre::CULL_NONE);
    bg_material_->setSceneBlending(Ogre::SBT_REPLACE);

    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();

    bg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
    bg_screen_rect_->setBoundingBox(aabInf);
    bg_screen_rect_->setMaterial(bg_material_);

    bg_scene_node_->attachObject(bg_screen_rect_);
    bg_scene_node_->setVisible(false);

    //overlay rectangle
    fg_screen_rect_ = new Ogre::Rectangle2D(true);
    fg_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    fg_material_ = bg_material_->clone(ss.str() + "fg");
    fg_screen_rect_->setBoundingBox(aabInf);
    fg_screen_rect_->setMaterial(fg_material_);

    fg_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    fg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);

    fg_scene_node_->attachObject(fg_screen_rect_);
    fg_scene_node_->setVisible(false);
  }

  updateAlpha();

  render_panel_ = new rviz_common::RenderPanel();
//  render_panel_->getRenderWindow()->addListener(this);
  // TODO(Martin-Idel-SI): Figure out if this is still needed
//  render_panel_->getRenderWindow()->setAutoUpdated(false);
//  render_panel_->getRenderWindow()->setActive(false);
  render_panel_->resize(640, 480);
  render_panel_->initialize(context_);

  setAssociatedWidget(render_panel_);

  // TODO(Martin-Idel-SI): Figure out if this is still needed
//  render_panel_->setAutoRender(false);
//  render_panel_->setOverlaysEnabled(false);
  rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_panel_->getRenderWindow())
    ->setNearClipDistance(0.01f);

//  caminfo_tf_filter_->connectInput(caminfo_sub_);
//  caminfo_tf_filter_->registerCallback(boost::bind(&CameraDisplay::caminfoCallback, this, _1));
  //context_->getFrameManager()->registerFilterForTransformStatusCheck(caminfo_tf_filter_, this);

  vis_bit_ = context_->visibilityBits()->allocBit();
  rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_panel_->getRenderWindow())
    ->setVisibilityMask(vis_bit_);

  visibility_property_ = new rviz_common::properties::DisplayGroupVisibilityProperty(
    vis_bit_, context_->getRootDisplayGroup(), this, "Visibility", true,
    "Changes the visibility of other Displays in the camera view.");

  visibility_property_->setIcon(
    rviz_common::loadPixmap("package://rviz/icons/visibility.svg", true));

  this->addChild(visibility_property_, 0);
}

void CameraDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent & evt)
{
  (void) evt;
  QString image_position = image_position_property_->getString();
  bg_scene_node_->setVisible(
    caminfo_ok_ && (image_position == BACKGROUND || image_position == BOTH));
  fg_scene_node_->setVisible(caminfo_ok_ && (image_position == OVERLAY || image_position == BOTH));

  // set view flags on all displays
  visibility_property_->update();
}

void CameraDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent & evt)
{
  (void) evt;
  bg_scene_node_->setVisible(false);
  fg_scene_node_->setVisible(false);
}

void CameraDisplay::onEnable()
{
  subscribe();
  // TODO(Martin-Idel-SI): Probably not needed?
//  render_panel_->getRenderWindow()->setActive(true);
}

void CameraDisplay::onDisable()
{
//  render_panel_->getRenderWindow()->setActive(false);
  unsubscribe();
  clear();
}

void CameraDisplay::subscribe()
{
  if ((!isEnabled()) || (topic_property_->getTopicStd().empty())) {
    return;
  }

  std::string target_frame = fixed_frame_.toStdString();
//  ImageDisplayBase::enableTFFilter(target_frame);

  RTDClass::subscribe();

//  std::string topic = topic_property_->getTopicStd();
//  std::string caminfo_topic = image_transport::getCameraInfoTopic(topic_property_->getTopicStd());

//  try {
//    caminfo_sub_.subscribe(update_nh_, caminfo_topic, 1);
//    setStatus(rviz_common::properties::StatusProperty::Ok, "Camera Info", "OK");
//  }
//  catch (ros::Exception & e) {
//    setStatus(rviz_common::properties::StatusProperty::Error, "Camera Info", QString("Error subscribing: ")
//      + e.what());
//  }
}

void CameraDisplay::unsubscribe()
{
  RTDClass::unsubscribe();
//  caminfo_sub_.unsubscribe();
}

void CameraDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();

  Ogre::Pass * pass = fg_material_->getTechnique(0)->getPass(0);
  if (pass->getNumTextureUnitStates() > 0) {
    Ogre::TextureUnitState * tex_unit = pass->getTextureUnitState(0);
    tex_unit->setAlphaOperation(Ogre::LBX_MODULATE, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha);
  } else {
    fg_material_->setAmbient(Ogre::ColourValue(0.0f, 1.0f, 1.0f, alpha));
    fg_material_->setDiffuse(Ogre::ColourValue(0.0f, 1.0f, 1.0f, alpha));
  }

  force_render_ = true;
  context_->queueRender();
}

void CameraDisplay::forceRender()
{
  force_render_ = true;
  context_->queueRender();
}

void CameraDisplay::updateQueueSize()
{

  // TODO(Martin-Idel-SI): Introduce QueueSizeProperty
//  caminfo_tf_filter_->setQueueSize((uint32_t) queue_size_property_->getInt());
//  ImageDisplayBase::updateQueueSize();
}

void CameraDisplay::clear()
{
  texture_.clear();
  force_render_ = true;
  context_->queueRender();

  new_caminfo_ = false;
  current_caminfo_.reset();

  // TODO(Martin-Idel-SI): Reintroduce correct topic. Was: "caminfo_sub_.getTopic()" instead of
  // "camera"
  setStatus(rviz_common::properties::StatusProperty::Warn, "Camera Info",
    "No CameraInfo received on [" + QString::fromStdString("camera") +
      "].  Topic may not exist.");
  setStatus(rviz_common::properties::StatusProperty::Warn, "Image", "No Image received");

  rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_panel_->getRenderWindow())
    ->setPosition(Ogre::Vector3(999999, 999999, 999999));
}

void CameraDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;
  try {
    if (texture_.update() || force_render_) {
      caminfo_ok_ = updateCamera();
      force_render_ = false;
    }
  }
  catch (UnsupportedImageEncoding & e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Image", e.what());
  }

  // TODO(Martin-Idel-SI): Figure out if this is still needed
//  render_panel_->getRenderWindow()->update();
}

bool CameraDisplay::updateCamera()
{
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  sensor_msgs::msg::Image::ConstSharedPtr image;
  {
    std::unique_lock<std::mutex> lock(caminfo_mutex_);
    info = current_caminfo_;
    image = texture_.getImage();
  }

  if (!info || !image) {
    return false;
  }

  if (!validateFloats(*info)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Camera Info",
      "Contains invalid floating point values (nans or infs)");
    return false;
  }

  // TODO(Martin-Idel-SI): Reenable exact mode
  // if we're in 'exact' time mode, only show image if the time is exactly right
//  ros::Time rviz_time = context_->getFrameManager()->getTime();
//  if (context_->getFrameManager()->getSyncMode() == rviz_common::FrameManager::SyncExact &&
//    rviz_time != image->header.stamp) {
//    std::ostringstream s;
//    s << "Time-syncing active and no image at timestamp " << rviz_time.toSec() << ".";
//    setStatus(rviz_common::properties::StatusProperty::Warn, "Time", s.str().c_str());
//    return false;
//  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  context_->getFrameManager()->getTransform(image->header.frame_id, image->header.stamp, position,
    orientation);

  //printf( "CameraDisplay:updateCamera(): pos = %.2f, %.2f, %.2f.\n", position.x, position.y, position.z );

  // convert vision (Z-forward) frame to ogre frame (Z-out)
  orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

  float img_width = info->width;
  float img_height = info->height;

  // If the image width is 0 due to a malformed caminfo, try to grab the width from the image.
  if (img_width == 0) {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Malformed CameraInfo on camera" << qPrintable(getName()) << ", "
      "width = 0");
    img_width = texture_.getWidth();
  }

  if (img_height == 0) {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Malformed CameraInfo on camera" << qPrintable(getName()) << ","
      " height = 0");
    img_height = texture_.getHeight();
  }

  if (img_height == 0.0 || img_width == 0.0) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Camera Info",
      "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0)");
    return false;
  }

  double fx = info->p[0];
  double fy = info->p[5];

  float win_width = render_panel_->width();
  float win_height = render_panel_->height();
  float zoom_x = zoom_property_->getFloat();
  float zoom_y = zoom_x;

  // Preserve aspect ratio
  if (win_width != 0 && win_height != 0) {
    float img_aspect = (img_width / fx) / (img_height / fy);
    float win_aspect = win_width / win_height;

    if (img_aspect > win_aspect) {
      zoom_y = zoom_y / img_aspect * win_aspect;
    } else {
      zoom_x = zoom_x / win_aspect * img_aspect;
    }
  }

  // Add the camera's translation relative to the left camera (from P[3]);
  double tx = -1 * (info->p[3] / fx);
  Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  position = position + (right * tx);

  double ty = -1 * (info->p[7] / fy);
  Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
  position = position + (down * ty);

  if (!rviz_common::validateFloats(position)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Camera Info",
      "CameraInfo/P resulted in an invalid position calculation (nans or infs)");
    return false;
  }

  rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_panel_->getRenderWindow())
    ->setPosition(position);
  rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_panel_->getRenderWindow())
    ->setOrientation(orientation);

  // calculate the projection matrix
  double cx = info->p[2];
  double cy = info->p[6];

  double far_plane = 100;
  double near_plane = 0.01;

  Ogre::Matrix4 proj_matrix;
  proj_matrix = Ogre::Matrix4::ZERO;

  proj_matrix[0][0] = 2.0 * fx / img_width * zoom_x;
  proj_matrix[1][1] = 2.0 * fy / img_height * zoom_y;

  proj_matrix[0][2] = 2.0 * (0.5 - cx / img_width) * zoom_x;
  proj_matrix[1][2] = 2.0 * (cy / img_height - 0.5) * zoom_y;

  proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
  proj_matrix[2][3] = -2.0 * far_plane * near_plane / (far_plane - near_plane);

  proj_matrix[3][2] = -1;

  rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_panel_->getRenderWindow())
    ->setCustomProjectionMatrix(true, proj_matrix);

  setStatus(rviz_common::properties::StatusProperty::Ok, "Camera Info", "OK");

#if 0
  static Axes* debug_axes = new Axes(scene_manager_, 0, 0.2, 0.01);
  debug_axes->setPosition(position);
  debug_axes->setOrientation(orientation);
#endif

  //adjust the image rectangles to fit the zoom & aspect ratio
  bg_screen_rect_->setCorners(-1.0f * zoom_x, 1.0f * zoom_y, 1.0f * zoom_x, -1.0f * zoom_y);
  fg_screen_rect_->setCorners(-1.0f * zoom_x, 1.0f * zoom_y, 1.0f * zoom_x, -1.0f * zoom_y);

  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  bg_screen_rect_->setBoundingBox(aabInf);
  fg_screen_rect_->setBoundingBox(aabInf);

  setStatus(rviz_common::properties::StatusProperty::Ok, "Time", "ok");
  setStatus(rviz_common::properties::StatusProperty::Ok, "Camera Info", "ok");

  return true;
}

void CameraDisplay::processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  texture_.addMessage(msg);
}

void CameraDisplay::caminfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  std::unique_lock<std::mutex> lock(caminfo_mutex_);
  current_caminfo_ = msg;
  new_caminfo_ = true;
}

void CameraDisplay::fixedFrameChanged()
{
  std::string targetFrame = fixed_frame_.toStdString();
//  caminfo_tf_filter_->setTargetFrame(targetFrame);
  RTDClass::fixedFrameChanged();
}

void CameraDisplay::reset()
{
  RTDClass::reset();
  clear();
}

}  // namespace displays

}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::CameraDisplay, rviz_common::Display)
