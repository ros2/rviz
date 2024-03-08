#include "rviz_default_plugins/displays/camera_info/camera_info_display.hpp"

#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreHardwarePixelBuffer.h>

#include <rviz_common/uniform_string_stream.hpp>
#include <rviz_common/properties/parse_color.hpp>

#include <image_geometry/pinhole_camera_model.hpp>

#include <string>
#include <memory>
#include <vector>

namespace rviz_default_plugins
{
namespace displays
{
TrianglePolygon::TrianglePolygon(
    Ogre::SceneManager* manager,
    Ogre::SceneNode* node,
    const cv::Point3d & O,
    const cv::Point3d & A,
    const cv::Point3d & B,
    const std::string& name,
    const Ogre::ColourValue& color,
    bool use_color,
    bool upper_triangle)
  {
    // uniq string is requred for name
    manual_ = manager->createManualObject();
    manual_->clear();
    manual_->begin(name,
                   Ogre::RenderOperation::OT_TRIANGLE_STRIP);
    manual_->position(O.x, O.y, O.z);
    if (upper_triangle) {
      manual_->textureCoord(0, 0);
    }
    else {
      manual_->textureCoord(1, 0);
    }
    if (use_color) {
      manual_->colour(color);
    }
    manual_->position(A.x, A.y, A.z);
    if (upper_triangle) {
      manual_->textureCoord(1, 0);
    }
    else {
      manual_->textureCoord(1, 1);
    }
    if (use_color) {
      manual_->colour(color);
    }
    manual_->position(B.x, B.y, B.z);
    if (upper_triangle) {
      manual_->textureCoord(0, 1);
    }
    else {
      manual_->textureCoord(0, 1);
    }
    if (use_color) {
      manual_->colour(color);
    }
    manual_->end();
    node->attachObject(manual_);
  }

  TrianglePolygon::~TrianglePolygon()
  {
    manual_->detachFromParent();
  }

CameraInfoDisplay::CameraInfoDisplay():
  image_updated_(true),
  rviz_common::MessageFilterDisplay<sensor_msgs::msg::CameraInfo>()
{
    ////////////////////////////////////////////////////////
    // initialize properties
    ////////////////////////////////////////////////////////
    far_clip_distance_property_ = new rviz_common::properties::FloatProperty(
      "far clip",
      1.0,
      "far clip distance from the origin of camera info",
      this, SLOT(updateFarClipDistance()));
    show_edges_property_ = new rviz_common::properties::BoolProperty(
      "show edges",
      true,
      "show edges of the region of the camera info",
      this, SLOT(updateShowEdges()));
    show_polygons_property_ = new rviz_common::properties::BoolProperty(
      "show polygons",
      true,
      "show polygons of the region of the camera info",
      this, SLOT(updateShowPolygons()));
    not_show_side_polygons_property_ = new rviz_common::properties::BoolProperty(
      "not show side polygons",
      true,
      "do not show polygons of the region of the camera info",
      this, SLOT(updateNotShowSidePolygons()));
    use_image_property_ = new rviz_common::properties::BoolProperty(
      "use image",
      false,
      "use image as texture",
      this, SLOT(updateUseImage()));

    // color image properties
    QRegExp color_filter("color|rgb|bgr|gray|mono");
    color_filter.setCaseSensitivity(Qt::CaseInsensitive);
    image_topic_property_ = new rviz_common::properties::RosFilteredTopicProperty(
      "Image Topic", "",
      "sensor_msgs/msg/Image",
      "Topic where updates to this camera info display are received",
      color_filter, this, SLOT(updateImageTopic()));
    image_topic_property_->hide();
    // image_transport_hints_property_ = new ImageTransportHintsProperty(
    //   "transport hints",
    //   "transport hint for image subscription",
    //   this, SLOT( updateImageTopic() ));
    // image_transport_hints_property_->hide();

    color_property_ = new rviz_common::properties::ColorProperty(
      "color",
      QColor(85, 255, 255),
      "color of CameraInfo",
      this, SLOT(updateColor()));
    edge_color_property_ = new rviz_common::properties::ColorProperty(
      "edge color",
      QColor(125, 125, 125),
      "edge color of CameraInfo",
      this, SLOT(updateEdgeColor()));
    alpha_property_ = new rviz_common::properties::FloatProperty(
      "alpha",
      0.5,
      "alpha blending value",
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
    camera_info_ = sensor_msgs::msg::CameraInfo::ConstPtr(); // reset to NULL
  }

  void CameraInfoDisplay::onInitialize()
  {
    auto rviz_ros_node_ = context_->getRosNodeAbstraction().lock();
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    image_topic_property_->initialize(rviz_ros_node_);
    updateColor();
    updateAlpha();
    updateFarClipDistance();
    updateShowPolygons();
    updateNotShowSidePolygons();
    updateShowEdges();
    updateImageTopic();
    updateUseImage();
    updateEdgeColor();
  }

  void CameraInfoDisplay::processMessage(
    const sensor_msgs::msg::CameraInfo::ConstPtr msg)
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
     if(!context_->getFrameManager()->getTransform(frame_id,
                                                   msg->header.stamp,
                                                   position,
                                                   quaternion)) {
    //    ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
    //               qPrintable( getName() ), msg->header.frame_id.c_str(),
    //               qPrintable( fixed_frame_ ));
     }
     scene_node_->setPosition(position);
     scene_node_->setOrientation(quaternion);
     camera_info_ = msg;        // store for caching
  }

  void CameraInfoDisplay::update(float wall_dt, float ros_dt)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (image_updated_) {
    //   ROS_DEBUG("image updated");
      if (!bottom_texture_.isNull()) {
        drawImageTexture();
        image_updated_ = false;
      }
    }
  }

  bool CameraInfoDisplay::isSameCameraInfo(
    const sensor_msgs::msg::CameraInfo::ConstPtr& msg)
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
      }
      else {
        return false;
      }
    }
    else {
      return false;
    }
  }

  void CameraInfoDisplay::addPointToEdge(
    const cv::Point3d & point)
  {
    Ogre::Vector3 p;
    p[0] = point.x;
    p[1] = point.y;
    p[2] = point.z;
    edges_->addPoint(p);
  }

  void CameraInfoDisplay::addPolygon(
    const cv::Point3d & O, const cv::Point3d & A, const cv::Point3d & B,
    std::string name, bool use_color, bool upper_triangle)
  {
    Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_);
    color.a = alpha_;
    std::shared_ptr<TrianglePolygon> triangle = std::make_shared<TrianglePolygon>(
                                     scene_manager_,
                                     scene_node_,
                                     O, A, B, name,
                                     color,
                                     use_color,
                                     upper_triangle);
    polygons_.push_back(triangle);
  }

  void CameraInfoDisplay::createTextureForBottom(int width, int height)
  {
    if (bottom_texture_.isNull()
        || bottom_texture_->getWidth() != width
        || bottom_texture_->getHeight() != height) {
      static uint32_t count = 0;
      rviz_common::UniformStringStream ss;
      ss << "CameraInfoDisplayPolygonBottom" << count++;
      material_bottom_
        = Ogre::MaterialManager::getSingletonPtr()->create(
          ss.str(),
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      bottom_texture_ = Ogre::TextureManager::getSingletonPtr()->createManual(
        material_bottom_->getName() + "Texture",        // name
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_A8R8G8B8, Ogre::TU_DEFAULT);
      material_bottom_->getTechnique(0)->getPass(0)->setColourWriteEnabled(true);
      Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_);
      color.a = alpha_;
      material_bottom_->getTechnique(0)->getPass(0)->setAmbient(color);
      material_bottom_->setReceiveShadows(false);
      material_bottom_->getTechnique(0)->setLightingEnabled(true);
      material_bottom_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
      material_bottom_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
      material_bottom_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
      material_bottom_->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);

      material_bottom_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE);
      material_bottom_->getTechnique(0)->getPass(0)->createTextureUnitState(bottom_texture_->getName());
      material_bottom_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    }
  }

  void CameraInfoDisplay::prepareMaterial()
  {
    if (texture_.isNull()) {
      // material
      static uint32_t count = 0;
      rviz_common::UniformStringStream ss;
      ss << "CameraInfoDisplayPolygon" << count++;
      material_
        = Ogre::MaterialManager::getSingletonPtr()->create(
          ss.str(),
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      texture_ = Ogre::TextureManager::getSingletonPtr()->createManual(
        material_->getName() + "Texture",        // name
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
      createTextureForBottom(640, 480);
    }
  }

  void CameraInfoDisplay::subscribeImage(std::string topic)
  {
    image_sub_.shutdown();
    if (topic.empty()) {
    //   ROS_WARN("topic name is empty");
      return;
    }
    auto rviz_ros_node_ = context_->getRosNodeAbstraction().lock();

    it_ = std::make_unique<image_transport::ImageTransport>(
        rviz_ros_node_->get_raw_node());
    image_sub_ = it_->subscribe(topic, 1, &CameraInfoDisplay::imageCallback, this);
                                // image_transport_hints_property_->getTransportHints());
  }

  void CameraInfoDisplay::drawImageTexture()
  {
    // bottom_texture_->getBuffer()->lock( Ogre::HardwareBuffer::HBL_NORMAL );
    // const Ogre::PixelBox& pixelBox
    //   = bottom_texture_->getBuffer()->getCurrentLock();
    // Ogre::uint8* pDest = static_cast<Ogre::uint8*> (pixelBox.data);

    // // Don't copy pixel-by-pixel image matrices.
    // // Just split matrix into channels, add needed alpha channel and merge back directly into buffer.
    // if (use_image_ && !image_.empty() &&
    //     bottom_texture_->getHeight() == image_.rows &&
    //     bottom_texture_->getWidth() == image_.cols) {
    // //   ROS_DEBUG("bottom_texture_->getHeight(): %u", bottom_texture_->getHeight());
    // //   ROS_DEBUG("bottom_texture_->getWidth(): %u", bottom_texture_->getWidth());
    // //   ROS_DEBUG("image_.rows: %d", image_.rows);
    // //   ROS_DEBUG("image_.cols: %d", image_.cols);

    //   std::vector<cv::Mat> splitted;
    //   cv::split(image_, splitted);
    //   // Swap channels RGB -> BGR for cv::merge.
    //   std::swap(splitted[0], splitted[2]);
    //   cv::Mat alpha(image_.rows, image_.cols, CV_8U, cv::Scalar(alpha_ * 255.0));
    //   splitted.push_back(alpha);
    //   cv::Mat boxMat(image_.rows, image_.cols, CV_8UC4, pDest);
    //   cv::merge(splitted, boxMat);
    // } else {
    //   memset(pDest, 0, bottom_texture_->getWidth() * bottom_texture_->getHeight());
    //   QImage Hud(pDest, bottom_texture_->getWidth(), bottom_texture_->getHeight(), QImage::Format_ARGB32);
    //   for (size_t j = 0; j < bottom_texture_->getHeight(); j++) {
    //     for (size_t i = 0; i < bottom_texture_->getWidth(); i++) {
    //       Hud.setPixel(i, j, color_.rgba());
    //     }
    //   }
    // }
    // bottom_texture_->getBuffer()->unlock();
  }

  // convert sensor_msgs::Image into cv::Mat
  void CameraInfoDisplay::imageCallback(
      const sensor_msgs::msg::Image::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // cv_bridge::CvImageConstPtr cv_ptr;
    // if (!camera_info_) {
    //   return;
    // }
    // try
    // {
    //   cv_ptr = cv_bridge::toCvShare(msg);
    //   cv::Mat im = cv_ptr->image.clone();
    //   if (msg->encoding == enc::BGRA8) {
    //     cv::cvtColor(im, im, cv::COLOR_BGRA2RGB);
    //   } else if (msg->encoding == enc::BGRA16) {
    //     im.convertTo(im, CV_8U, 1 / 256.0);
    //     cv::cvtColor(im, im, cv::COLOR_BGRA2RGB);
    //   } else if (msg->encoding == enc::BGR8) {
    //     cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    //   } else if (msg->encoding == enc::BGR16) {
    //     im.convertTo(im, CV_8U, 1 / 256.0);
    //     cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    //   } else if (msg->encoding == enc::RGBA8) {
    //     cv::cvtColor(im, im, cv::COLOR_RGBA2RGB);
    //   } else if (msg->encoding == enc::RGBA16) {
    //     im.convertTo(im, CV_8U, 1 / 256.0);
    //     cv::cvtColor(im, im, cv::COLOR_RGBA2RGB);
    //   } else if (msg->encoding == enc::RGB8) {
    //     // nothing
    //   } else if (msg->encoding == enc::RGB16) {
    //     im.convertTo(im, CV_8U, 1 / 256.0);
    //   } else if (msg->encoding == enc::MONO8) {
    //     cv::cvtColor(im, im, cv::COLOR_GRAY2RGB);
    //   } else if (msg->encoding == enc::MONO16) {
    //     im.convertTo(im, CV_8U, 1 / 256.0);
    //     cv::cvtColor(im, im, cv::COLOR_GRAY2RGB);
    //   } else {
    //     ROS_ERROR("[CameraInfoDisplay] Not supported image encodings %s.", msg->encoding.c_str());
    //     return;
    //   }

    //   int roi_height = camera_info_->roi.height ? camera_info_->roi.height : camera_info_->height;
    //   int roi_width = camera_info_->roi.width ? camera_info_->roi.width : camera_info_->width;
    //   if (camera_info_->binning_y > 0) {
    //     roi_height /= camera_info_->binning_y;
    //   }
    //   if (camera_info_->binning_x > 0) {
    //     roi_width /= camera_info_->binning_x;
    //   }

    //   if (im.cols == camera_info_->width && im.rows == camera_info_->height) {
    //     cv::Rect roi(camera_info_->roi.x_offset, camera_info_->roi.y_offset,
    //                  camera_info_->roi.width ? camera_info_->roi.width : camera_info_->width,
    //                  camera_info_->roi.height ? camera_info_->roi.height : camera_info_->height);
    //     image_ = cv::Mat(im, roi).clone();
    //   } else if (im.cols == roi_width && im.rows == roi_height) {
    //     image_ = im.clone();
    //   } else {
    //     ROS_ERROR("[CameraInfoDisplay] Invalid image size (w, h) = (%d, %d), expected (w, h) = (%d, %d) or (%d, %d) (ROI size)",
    //               im.cols, im.rows,
    //               camera_info_->width, camera_info_->height,
    //               roi_width, roi_height);
    //     return;
    //   }

    //   // check the size of bottom texture
    //   if (bottom_texture_.isNull()
    //       || bottom_texture_->getWidth() != image_.cols
    //       || bottom_texture_->getHeight() != image_.rows) {
    //     createTextureForBottom(image_.cols, image_.rows);
    //     if (camera_info_) {
    //       createCameraInfoShapes(camera_info_);
    //     }
    //   }
    //   image_updated_ = true;
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //   ROS_ERROR("cv_bridge exception: %s", e.what());
    // }
  }

  void CameraInfoDisplay::createCameraInfoShapes(
    const sensor_msgs::msg::CameraInfo::ConstPtr& msg)
  {
    polygons_.clear();
    if (edges_) {
      edges_->clear();
    }
    image_geometry::PinholeCameraModel model;
    bool model_success_p = model.fromCameraInfo(msg);
    if (!model_success_p) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Camera Info", "Failed to create camera model from msg");
    //   ROS_ERROR("failed to create camera model");
      return;
    }
    // fx and fy should not be equal 0.
    if (model.fx() == 0.0 || model.fy() == 0.0) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Camera Info", "Invalid intrinsic matrix");
    //   ROS_ERROR_STREAM("camera model have invalid intrinsic matrix " << model.intrinsicMatrix());
      return;
    }
    setStatus(rviz_common::properties::StatusProperty::Ok, "Camera Info", "OK");

    ////////////////////////////////////////////////////////
    // initialize BillboardLine
    ////////////////////////////////////////////////////////
    if (!edges_) {
      edges_.reset(new rviz_rendering::BillboardLine(context_->getSceneManager(),
                                           scene_node_));
      edges_->setLineWidth(0.01);
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
    cv::Point3d A = model.projectPixelTo3dRay(cv::Point2d(a.x, a.y));
    cv::Point3d B = model.projectPixelTo3dRay(cv::Point2d(b.x, b.y));
    cv::Point3d C = model.projectPixelTo3dRay(cv::Point2d(c.x, c.y));
    cv::Point3d D = model.projectPixelTo3dRay(cv::Point2d(d.x, d.y));

    cv::Point3d scaled_A = A * far_clip_distance_;
    cv::Point3d scaled_B = B * far_clip_distance_;
    cv::Point3d scaled_C = C * far_clip_distance_;
    cv::Point3d scaled_D = D * far_clip_distance_;

    cv::Point3d O(0, 0, 0);

    ////////////////////////////////////////////////////////
    // build polygons
    ////////////////////////////////////////////////////////
    if (show_polygons_) {
      ////////////////////////////////////////////////////////
      // setup color for polygons
      ////////////////////////////////////////////////////////
      Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_);
      color.a = alpha_;
      prepareMaterial();
      if (!not_show_side_polygons_) {
        material_->getTechnique(0)->getPass(0)->setAmbient(color);
        {
          texture_->getBuffer()->lock( Ogre::HardwareBuffer::HBL_NORMAL );
          const Ogre::PixelBox& pixelBox
            = texture_->getBuffer()->getCurrentLock();
          Ogre::uint8* pDest = static_cast<Ogre::uint8*> (pixelBox.data);
          memset(pDest, 0, 1);
          QImage Hud(pDest, 1, 1, QImage::Format_ARGB32 );
          Hud.setPixel(0, 0, color_.rgba());
          texture_->getBuffer()->unlock();
        }
        addPolygon(O, scaled_B, scaled_A, material_->getName(), true, true);
        addPolygon(O, scaled_C, scaled_B, material_->getName(), true, true);
        addPolygon(O, scaled_D, scaled_C, material_->getName(), true, true);
        addPolygon(O, scaled_A, scaled_D, material_->getName(), true, true);
      }
      // bottom
      drawImageTexture();

      addPolygon(scaled_A, scaled_B, scaled_D, material_bottom_->getName(), false, true);
      addPolygon(scaled_B, scaled_C, scaled_D, material_bottom_->getName(), false, false);
    }
    ////////////////////////////////////////////////////////
    // build edges
    ////////////////////////////////////////////////////////
    if (show_edges_) {
      edges_->clear();
      edges_->setMaxPointsPerLine(2);
      edges_->setNumLines(8);
      edges_->setColor(edge_color_.red() / 255.0,
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

  ////////////////////////////////////////////////////////
  // Properties updating functions
  ////////////////////////////////////////////////////////
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
    }
    else {
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

  void CameraInfoDisplay::updateImageTopic()
  {
    // if (use_image_) {
    //   std::string topic = image_topic_property_->getStdString();
    //   std::cout << "topic " << topic << std::endl;
    //   // subscribeImage(topic);
    // } else {
    //   // image_sub_.shutdown();
    //   // // Set image_updated_ true in order to clear the bottom texture in update() method.
    //   // image_updated_ = true;
    // }
  }

  void CameraInfoDisplay::updateUseImage()
  {
    use_image_ = use_image_property_->getBool();
    if (use_image_) {
      image_topic_property_->show();
    //   image_transport_hints_property_->show();
    }
    else {
      image_topic_property_->hide();
    //   image_transport_hints_property_->hide();
    }
    updateImageTopic();
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
