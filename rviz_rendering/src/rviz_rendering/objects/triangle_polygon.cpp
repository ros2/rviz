#include "rviz_rendering/objects/triangle_polygon.hpp"

#include <string>

#include <OgreColourValue.h>
#include <OgreManualObject.h>
#include <OgreRenderOperation.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreVector.h>

namespace rviz_rendering
{
TrianglePolygon::TrianglePolygon(
  Ogre::SceneManager * manager,
  Ogre::SceneNode * node,
  const Ogre::Vector3 & O,
  const Ogre::Vector3 & A,
  const Ogre::Vector3 & B,
  const std::string & name,
  const Ogre::ColourValue & color,
  bool use_color,
  bool upper_triangle)
{
  // uniq string is requred for name
  manual_ = manager->createManualObject();
  manual_->clear();
  manual_->begin(
    name,
    Ogre::RenderOperation::OT_TRIANGLE_STRIP);
  manual_->position(O.x, O.y, O.z);
  if (upper_triangle) {
    manual_->textureCoord(0, 0);
  } else {
    manual_->textureCoord(1, 0);
  }
  if (use_color) {
    manual_->colour(color);
  }
  manual_->position(A.x, A.y, A.z);
  if (upper_triangle) {
    manual_->textureCoord(1, 0);
  } else {
    manual_->textureCoord(1, 1);
  }
  if (use_color) {
    manual_->colour(color);
  }
  manual_->position(B.x, B.y, B.z);
  if (upper_triangle) {
    manual_->textureCoord(0, 1);
  } else {
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
}  // namespace rviz_rendering
