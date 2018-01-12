/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

// Adapted from: http://www.ogre3d.org/wiki/index.php/MovableText
//          now: http://www.ogre3d.org/tikiwiki/tiki-index.php?page=MovableText
// Original authors:
/*
 * File: MovableText.h
 *
 * description: This create a billboarding object that displays a text.
 *
 * @author  2003 by cTh see gavocanov@rambler.ru
 * @update  2006 by barraq see nospam@barraquand.com
 */

#ifndef RVIZ_RENDERING__MOVABLE_TEXT_HPP_
#define RVIZ_RENDERING__MOVABLE_TEXT_HPP_

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <OgreMovableObject.h>
#include <OgreQuaternion.h>
#include <OgreRenderable.h>
#include <OgreSharedPtr.h>
#include <OgreVector3.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "rviz_rendering/visibility_control.hpp"

namespace Ogre
{
class Camera;
class Font;
class RenderQueue;
}  // namespace Ogre

namespace rviz_rendering
{

class RVIZ_RENDERING_LOCAL MovableText : public Ogre::MovableObject, public Ogre::Renderable
{
public:
  enum RVIZ_RENDERING_PUBLIC HorizontalAlignment
  {
    H_LEFT, H_CENTER
  };
  enum RVIZ_RENDERING_PUBLIC VerticalAlignment
  {
    V_BELOW, V_ABOVE, V_CENTER
  };

public:
  explicit MovableText(
    const Ogre::String & caption,
    const Ogre::String & fontName = "Liberation Sans",
    Ogre::Real charHeight = 1.0,
    const Ogre::ColourValue & color = Ogre::ColourValue::White);
  ~MovableText() override;

  void setFontName(const Ogre::String & font_name);
  void setCaption(const Ogre::String & caption);
  RVIZ_RENDERING_PUBLIC
  void setColor(const Ogre::ColourValue & color);
  RVIZ_RENDERING_PUBLIC
  void setCharacterHeight(Ogre::Real height);
  RVIZ_RENDERING_PUBLIC
  void setLineSpacing(Ogre::Real height);
  RVIZ_RENDERING_PUBLIC
  void setSpaceWidth(Ogre::Real width);
  RVIZ_RENDERING_PUBLIC
  void setTextAlignment(
    const HorizontalAlignment & horizontal_alignment,
    const VerticalAlignment & vertical_alignment);
  void setGlobalTranslation(Ogre::Vector3 translation);
  void setLocalTranslation(Ogre::Vector3 translation);
  void showOnTop(bool show = true);

  const Ogre::String & getFontName() const
  {
    return font_name_;
  }
  const Ogre::String & getCaption() const
  {
    return caption_;
  }
  const Ogre::ColourValue & getColor() const
  {
    return color_;
  }
  Ogre::Real getCharacterHeight() const
  {
    return char_height_;
  }
  Ogre::Real getSpaceWidth() const
  {
    return space_width_;
  }
  Ogre::Vector3 getGlobalTranslation() const
  {
    return global_translation_;
  }
  Ogre::Vector3 getLocalTranslation() const
  {
    return local_translation_;
  }
  bool getShowOnTop() const
  {
    return on_top_;
  }
  const Ogre::AxisAlignedBox & getBoundingBox() const override
  {
    return bounding_box_;
  }
  Ogre::Real getBoundingRadius() const override
  {
    return radius_;
  }
  const Ogre::MaterialPtr & getMaterial() const override
  {
    assert(material_);
    return material_;
  }

  void getRenderOperation(Ogre::RenderOperation & op) override;

  void
  visitRenderables(Ogre::Renderable::Visitor * visitor, bool debug_renderables) override;

protected:
  void setupGeometry();
  unsigned int calculateVertexCount() const;
  void setupRenderOperation();
  Ogre::HardwareVertexBufferSharedPtr setupHardwareBuffers() const;
  void calculateTotalDimensionsForPositioning(float & total_height, float & total_width) const;
  float getVerticalStartFromVerticalAlignment(float total_height) const;
  float getLineStartFromHorizontalAlignment(float total_width) const;
  void fillVertexBuffer(
    Ogre::HardwareVertexBufferSharedPtr & position_and_texture_buffer,
    float top, float starting_left);

  void updateColors();

  void getWorldTransforms(Ogre::Matrix4 * xform) const override;
  Ogre::Real getSquaredViewDepth(const Ogre::Camera * cam) const override
  {
    (void) cam;
    return 0;
  }
  const Ogre::Quaternion & getWorldOrientation() const;
  const Ogre::Vector3 & getWorldPosition() const;
  const Ogre::String & getMovableType() const override
  {
    static Ogre::String movType = "MovableText";
    return movType;
  }

  void _notifyCurrentCamera(Ogre::Camera * camera) override;
  void _updateRenderQueue(Ogre::RenderQueue * queue) override;

  const Ogre::LightList & getLights() const override
  {
    return light_list_;
  }

  void fillColorBuffer(Ogre::RGBA color) const;

  void update();

private:
  Ogre::String font_name_;
  Ogre::String name_;
  Ogre::String caption_;
  HorizontalAlignment horizontal_alignment_;
  VerticalAlignment vertical_alignment_;

  Ogre::ColourValue color_;
  Ogre::RenderOperation render_operation_;
  Ogre::AxisAlignedBox bounding_box_;
  Ogre::Real radius_;

  Ogre::Real char_height_;
  Ogre::Real line_spacing_;
  Ogre::Real space_width_;

  bool needs_update_;
  bool needs_color_update_;
  bool on_top_;

  Ogre::Vector3 global_translation_;
  Ogre::Vector3 local_translation_;

  Ogre::Camera * camera_;
  Ogre::Font * font_;
  Ogre::MaterialPtr material_;

  Ogre::LightList light_list_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__MOVABLE_TEXT_HPP_
