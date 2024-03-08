/*
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__CAMERA_INFO__CAMERA_INFO_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__CAMERA_INFO__CAMERA_INFO_DISPLAY_HPP_

#ifndef Q_MOC_RUN

#include <memory>
#include <string>

#include "rviz_default_plugins/visibility_control.hpp"

#include <image_transport/image_transport.hpp>

#include <opencv2/core/core.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

namespace rviz_default_plugins
{
namespace displays
{
class TrianglePolygon
{
public:
  TrianglePolygon(Ogre::SceneManager* manager,
                  Ogre::SceneNode* node,
                  const cv::Point3d & O,
                  const cv::Point3d & A,
                  const cv::Point3d & B,
                  const std::string& name,
                  const Ogre::ColourValue& color,
                  bool use_color,
                  bool upper_triangle);
  virtual ~TrianglePolygon();
protected:
  Ogre::ManualObject* manual_;
  Ogre::SceneManager* manager_;
};

/**
 * \class CameraInfoDisplay
 *
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC CameraInfoDisplay
  : public rviz_common::MessageFilterDisplay<sensor_msgs::msg::CameraInfo>
{
  Q_OBJECT
public:
    CameraInfoDisplay();
    ~CameraInfoDisplay() override;

  // Overrides of public virtual functions from the Display class.
  void onInitialize() override;
  void reset() override;

  protected:
    void processMessage(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    ////////////////////////////////////////////////////////
    // methods
    ///////////////////////////////////////////////////////
    void update(float wall_dt, float ros_dt) override;
    bool isSameCameraInfo(
      const sensor_msgs::msg::CameraInfo::ConstPtr& camera_info);
    void createCameraInfoShapes(
      const sensor_msgs::msg::CameraInfo::ConstPtr& camera_info);
    void addPointToEdge(
      const cv::Point3d & point);
    void addPolygon(
      const cv::Point3d & O, const cv::Point3d & A, const cv::Point3d & B, std::string name,
      bool use_color, bool upper_triangle);
    void prepareMaterial();
    void createTextureForBottom(int width, int height);
    void imageCallback(const sensor_msgs::msg::Image::ConstPtr& msg);
    void drawImageTexture();
    void subscribeImage(std::string topic);
    /////////////////////////////////////////////////////////
    // variables
    ////////////////////////////////////////////////////////
    std::vector<std::shared_ptr<TrianglePolygon>> polygons_;
    std::shared_ptr<rviz_rendering::BillboardLine> edges_;
    sensor_msgs::msg::CameraInfo::ConstPtr camera_info_;
    Ogre::MaterialPtr material_;
    Ogre::TexturePtr texture_;
    Ogre::MaterialPtr material_bottom_;
    Ogre::TexturePtr bottom_texture_;
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_sub_;
    std::mutex mutex_;
    ////////////////////////////////////////////////////////
    // variables updated by rviz properties
    ////////////////////////////////////////////////////////
    double alpha_;
    double far_clip_distance_;
    QColor color_;
    QColor edge_color_;
    bool show_polygons_;
    bool show_edges_;
    bool use_image_;
    bool image_updated_;
    bool not_show_side_polygons_;
    // cv::Mat image_;
    ////////////////////////////////////////////////////////
    // properties
    ////////////////////////////////////////////////////////
    // ImageTransportHintsProperty* image_transport_hints_property_;
    rviz_common::properties::FloatProperty* far_clip_distance_property_;
    rviz_common::properties::FloatProperty* alpha_property_;
    rviz_common::properties::ColorProperty* color_property_;
    rviz_common::properties::ColorProperty* edge_color_property_;
    rviz_common::properties::BoolProperty* show_polygons_property_;
    rviz_common::properties::BoolProperty* not_show_side_polygons_property_;
    rviz_common::properties::BoolProperty* use_image_property_;
    rviz_common::properties::RosTopicProperty* image_topic_property_;
    rviz_common::properties::BoolProperty* show_edges_property_;

  protected Q_SLOTS:
    void updateFarClipDistance();
    void updateAlpha();
    void updateColor();
    void updateShowEdges();
    void updateShowPolygons();
    void updateNotShowSidePolygons();
    void updateImageTopic();
    void updateUseImage();
    void updateEdgeColor();
};
}  // namespace displays
}  // namespace rviz_default_plugins
#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__CAMERA_INFO__CAMERA_INFO_DISPLAY_HPP_
