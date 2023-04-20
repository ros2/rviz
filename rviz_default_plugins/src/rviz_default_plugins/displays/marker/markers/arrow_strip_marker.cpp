// TODO: Rviz 2 Port
#include "rviz_default_plugins/displays/marker/markers/arrow_strip_marker.hpp"

// #include <rviz/ogre_helpers/ogre_vector.h>
// #include <OgreQuaternion.h>
// #include <OgreSceneNode.h>
// #include <OgreSceneManager.h>
// #include <OgreEntity.h>

// #include <rviz/default_plugin/marker_display.h>
// #include <rviz/default_plugin/markers/marker_selection_handler.h>
// #include <rviz/display_context.h>
// #include <rviz/ogre_helpers/arrow.h>
// #include <rviz/ogre_helpers/shape.h>
// #include <rviz/validate_floats.h>
#include "rviz_common/msg_conversions.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{
ArrowStripMarker::ArrowStripMarker(MarkerDisplay* owner, rviz_common::DisplayContext* context, Ogre::SceneNode* parent_node)
  : MarkerBase(owner, context, parent_node)
{
}

void ArrowStripMarker::clearArrows() {
  for (Arrow* arrow : arrows_)
  {
    delete arrow;
  }
  arrows_.clear();
}

ArrowStripMarker::~ArrowStripMarker()
{
  clearArrows();
}

void ArrowStripMarker::onNewMessage(const MarkerConstSharedPtr & old_message, const MarkerConstSharedPtr & new_message)
{
  (void) old_message;

  assert(new_message->type == visualization_msgs::msg::Marker::ARROW_STRIP);

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  if (!transform(new_message, pos, orient, scale)){   // NOLINT: is super class method
    scene_node_->setVisible(false);
    return;
  }

  scene_node_->setVisible(true);
  setPosition(pos);
  setOrientation(orient);

  clearArrows();

  handler_.reset(new MarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id), context_));
  if (new_message->points.size() < 2) {
    printErrorMessage();
    scene_node_->setVisible(false);
    return;
  }

  // if scale.x and scale.y are 0, then nothing is shown
  if (owner_ && (new_message->scale.x + new_message->scale.y == 0.0f)) {
    owner_->setMarkerStatus(
      getID(), rviz_common::properties::StatusProperty::Warn, "Scale of 0 in both x and y");
      return;
  }

  for (int i = 0; i < new_message->points.size() - 1; i++) {
    Ogre::Vector3 start = rviz_common::pointMsgToOgre(new_message->points.at(i));
    Ogre::Vector3 end = rviz_common::pointMsgToOgre(new_message->points.at(i+1));
    Arrow arrow = new Arrow(context_->getSceneManager(), scene_node_);
    arrow_->setEndpoints(start, end);
    arrow_->setShaftDiameter(new_message->scale.x);
    arrow_->setHeadDiameter(new_message->scale.y);
    float head_length = std::clamp(new_message->scale.z, 0, arrow_->getLength());
    if (head_length > 0.0) {
      arrow_->setShaftHeadRatio(head_length - arrow_->getLength(), head_length)
    } else {
      arrow_->setShaftHeadRatio(3, 1); // default 3:1 ratio from arrow.hpp
    }
    arrow_->setColor(rviz_common::colorMsgToOgre(new_message.color));
    handler_->addTrackedObjects(arrows_.back()->getSceneNode());
    arrow_.push_back(arrow);
  }
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins