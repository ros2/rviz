#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__ARROW_STRIP_MARKER_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__ARROW_STRIP_MARKER_HPP_

#include "marker_base.hpp"

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre
{
class SceneNode;
}  // namespace Ogre
namespace rviz_common
{
class DisplayContext;
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC ArrowStripMarker : public MarkerBase
{
public:
  ArrowStripMarker(MarkerCommon* owner, rviz_common::DisplayContext* context, Ogre::SceneNode* parent_node);
  ~ArrowStripMarker() override = default;

protected:
  void onNewMessage(const MarkerConstSharedPtr & old_message, const MarkerConstSharedPtr & new_message) override;
  std::vector<std::unique_ptr<rviz_rendering::Arrow>> arrows_;
};

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins

#endif