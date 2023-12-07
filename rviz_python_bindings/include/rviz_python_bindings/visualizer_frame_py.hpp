#ifndef VISUALIZER_FRAME_PY_HPP
#define VISUALIZER_FRAME_PY_HPP

#include "rviz_common/visualization_frame.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

namespace rviz_common
{

class VisualizerFramePy : public VisualizationFrame
{
protected:
  std::unique_ptr<rviz_common::ros_integration::RosClientAbstraction> ros_client_abstraction_;

public:
  explicit VisualizerFramePy(
    QWidget * parent = nullptr);

  ~VisualizerFramePy()
  {
    ros_client_abstraction_->shutdown();
  }

  bool node_ok();

  void initialize(const QString & display_config_file = "");
};
}

#endif
