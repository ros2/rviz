/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

// Adapted from: http://www.ogre3d.org/wiki/index.php/MovableText
//          now: http://www.ogre3d.org/tikiwiki/tiki-index.php?page=MovableText
// Original authors:
/*
 * File: MovableText.cpp
 *
 * description: This creates a billboarding object that displays a text.
 *
 * @author  2003 by cTh see gavocanov@rambler.ru
 * @update  2006 by barraq see nospam@barraquand.com
 */

#include "rviz_rendering/objects/movable_text.hpp"

#include <algorithm>
#include <limits>
#include <sstream>
#include <string>

#include <OgreCamera.h>
#include <OgreHardwareBufferManager.h>
#include <OgreMaterialManager.h>
#include <OgreQuaternion.h>
#include <OgreRoot.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>
#include <Overlay/OgreFont.h>  // NOLINT: cpplint cannot handle include order here
#include <Overlay/OgreFontManager.h>  // NOLINT: cpplint cannot handle include order here

#define POS_TEX_BINDING    0
#define COLOUR_BINDING     1

#define MATERIAL_GROUP "rviz_rendering"

namespace rviz_rendering
{

const float effective_char_height_factor = 2.0f;

MovableText::MovableText(
  const Ogre::String & caption,
  const Ogre::String & fontName,
  Ogre::Real charHeight,
  const Ogre::ColourValue & color)
: font_name_(fontName),
  caption_(caption),
  horizontal_alignment_(H_LEFT),
  vertical_alignment_(V_BELOW),
  color_(color),
  char_height_(charHeight),
  line_spacing_(0.01f),
  space_width_(0),
  needs_color_update_(true),
  on_top_(false),
  global_translation_(0.0f),
  local_translation_(0.0f),
  font_(nullptr)
{
  static int count = 0;
  std::stringstream ss;
  ss << "MovableText" << count++;
  name_ = ss.str();

  mRenderOp.vertexData = nullptr;
  this->setFontName(font_name_);
  // Set a reasonable default space width
  space_width_ = font_->getGlyphAspectRatio('A') * char_height_ * effective_char_height_factor;
  this->setupGeometry();
}

MovableText::~MovableText()
{
  delete mRenderOp.vertexData;
  if (material_) {
    Ogre::MaterialManager::getSingletonPtr()->remove(material_->getName(), MATERIAL_GROUP);
  }
}

void MovableText::setFontName(const Ogre::String & font_name)
{
  if (Ogre::MaterialManager::getSingletonPtr()
    ->resourceExists(name_ + "Material", MATERIAL_GROUP))
  {
    Ogre::MaterialManager::getSingleton().remove(name_ + "Material", MATERIAL_GROUP);
  }

  if (font_name_ != font_name || !material_ || !font_) {
    font_name_ = font_name;
    font_ = Ogre::FontManager::getSingleton().getByName(font_name_, MATERIAL_GROUP).get();
    if (!font_) {
      throw Ogre::Exception(
              Ogre::Exception::ERR_ITEM_NOT_FOUND, "Could not find font " +
              font_name, "MovableText::setFontName");
    }

    font_->load();
    if (material_) {
      Ogre::MaterialManager::getSingletonPtr()->remove(material_->getName(), MATERIAL_GROUP);
      material_.reset();
    }

    material_ = font_->getMaterial()->clone(name_ + "Material");
    if (!material_->isLoaded()) {
      material_->load();
    }

    material_->setDepthCheckEnabled(!on_top_);
    material_->setDepthBias(1.0, 1.0);
    material_->setDepthWriteEnabled(on_top_);
    material_->setLightingEnabled(false);
    needs_update_ = true;
  }
}

void MovableText::setCaption(const Ogre::String & caption)
{
  if (caption != caption_) {
    caption_ = caption;
    needs_update_ = true;
  }
}

void MovableText::setColor(const Ogre::ColourValue & color)
{
  if (color != color_) {
    color_ = color;
    needs_color_update_ = true;
  }
}

void MovableText::setCharacterHeight(Ogre::Real height)
{
  if (height != char_height_) {
    char_height_ = height;
    needs_update_ = true;
  }
}

void MovableText::setLineSpacing(Ogre::Real height)
{
  if (height != line_spacing_) {
    line_spacing_ = height;
    needs_update_ = true;
  }
}

void MovableText::setSpaceWidth(Ogre::Real width)
{
  if (width != space_width_) {
    space_width_ = width;
    needs_update_ = true;
  }
}

void MovableText::setTextAlignment(
  const HorizontalAlignment & horizontal_alignment,
  const VerticalAlignment & vertical_alignment)
{
  if (horizontal_alignment_ != horizontal_alignment) {
    horizontal_alignment_ = horizontal_alignment;
    needs_update_ = true;
  }
  if (vertical_alignment_ != vertical_alignment) {
    vertical_alignment_ = vertical_alignment;
    needs_update_ = true;
  }
}

void MovableText::setGlobalTranslation(Ogre::Vector3 translation)
{
  global_translation_ = translation;
}

void MovableText::setLocalTranslation(Ogre::Vector3 translation)
{
  local_translation_ = translation;
}

void MovableText::showOnTop(bool show)
{
  if (on_top_ != show && material_) {
    on_top_ = show;
    material_->setDepthBias(1.0, 1.0);
    material_->setDepthCheckEnabled(!on_top_);
    material_->setDepthWriteEnabled(on_top_);
  }
}

struct TextBuffer
{
  float * buffer_;
  Ogre::Vector3 min_;
  Ogre::Vector3 max_;
  Ogre::Real max_squared_radius_;
  float top_;
  float left_;
  Ogre::Font::UVRect text_coords_;

  explicit TextBuffer(float * buffer)
  : buffer_(buffer),
    min_(std::numeric_limits<float>::max()),
    max_(std::numeric_limits<float>::min()),
    max_squared_radius_(0),
    top_(0),
    left_(0)
  {
  }

  void addTopLeft()
  {
    addPositionToBuffer(0, 0);
    addTextureToBuffer(text_coords_.left, text_coords_.top);
  }

  void addBottomLeft(float char_height)
  {
    addPositionToBuffer(0, 2.0f * char_height);
    addTextureToBuffer(text_coords_.left, text_coords_.bottom);
  }

  void addTopRight(float char_width)
  {
    addPositionToBuffer(2.0f * char_width, 0);
    addTextureToBuffer(text_coords_.right, text_coords_.top);
  }

  void addBottomRight(float char_widht, float char_height)
  {
    addPositionToBuffer(2.0f * char_widht, 2.0f * char_height);
    addTextureToBuffer(text_coords_.right, text_coords_.bottom);
  }

private:
  void addPositionToBuffer(float plus_left, float minus_top)
  {
    Ogre::Vector3 current_position = Ogre::Vector3(left_ + plus_left, top_ - minus_top, 0.0);
    *buffer_++ = current_position.x;
    *buffer_++ = current_position.y;
    *buffer_++ = current_position.z;

    min_.makeFloor(current_position);
    max_.makeCeil(current_position);
    max_squared_radius_ = std::max(max_squared_radius_, current_position.squaredLength());
  }

  void addTextureToBuffer(float texture_x, float texture_y)
  {
    *buffer_++ = texture_x;
    *buffer_++ = texture_y;
  }
};

void MovableText::setupGeometry()
{
  assert(font_);
  assert(material_);

  if (caption_.empty()) {
    return;
  }

  setupRenderOperation();
  Ogre::HardwareVertexBufferSharedPtr position_and_texture_buffer = setupHardwareBuffers();

  float total_height;
  float total_width;
  calculateTotalDimensionsForPositioning(total_height, total_width);

  float starting_left = getLineStartFromHorizontalAlignment(total_width);
  float starting_top = getVerticalStartFromVerticalAlignment(total_height);

  fillVertexBuffer(position_and_texture_buffer, starting_top, starting_left);

  if (needs_color_update_) {
    this->updateColors();
  }

  needs_update_ = false;
}

void MovableText::setupRenderOperation()
{
  unsigned int vertex_count = calculateVertexCount();

  if (mRenderOp.vertexData) {
    delete mRenderOp.vertexData;
    mRenderOp.vertexData = nullptr;
    needs_color_update_ = true;
  }

  mRenderOp.vertexData = new Ogre::VertexData();
  mRenderOp.indexData = nullptr;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = vertex_count;
  mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  mRenderOp.useIndexes = false;
}

unsigned int MovableText::calculateVertexCount() const
{
  unsigned int vertex_count = 0;
  for (auto & character : caption_) {
    if ((character != ' ') && (character != '\n')) {
      vertex_count += 6;
    }
  }
  return vertex_count;
}

Ogre::HardwareVertexBufferSharedPtr MovableText::setupHardwareBuffers() const
{
  Ogre::VertexDeclaration * declaration = mRenderOp.vertexData->vertexDeclaration;
  Ogre::VertexBufferBinding * bind = mRenderOp.vertexData->vertexBufferBinding;
  size_t offset = 0;

  // create/bind positions/texture coordinates buffer
  if (!declaration->findElementBySemantic(Ogre::VES_POSITION)) {
    declaration->addElement(POS_TEX_BINDING, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  }

  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  if (!declaration->findElementBySemantic(Ogre::VES_TEXTURE_COORDINATES)) {
    declaration->addElement(
      POS_TEX_BINDING, offset, Ogre::VET_FLOAT2,
      Ogre::VES_TEXTURE_COORDINATES, 0);
  }

  Ogre::HardwareVertexBufferSharedPtr position_and_texture_buffer =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
    declaration->getVertexSize(POS_TEX_BINDING),
    mRenderOp.vertexData->vertexCount,
    Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);
  bind->setBinding(POS_TEX_BINDING, position_and_texture_buffer);

  // Colours - store these in a separate buffer because they change less often
  if (!declaration->findElementBySemantic(Ogre::VES_DIFFUSE)) {
    declaration->addElement(COLOUR_BINDING, 0, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
  }

  Ogre::HardwareVertexBufferSharedPtr color_buffer =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
    declaration->getVertexSize(COLOUR_BINDING),
    mRenderOp.vertexData->vertexCount,
    Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);
  bind->setBinding(COLOUR_BINDING, color_buffer);
  return position_and_texture_buffer;
}

void
MovableText::calculateTotalDimensionsForPositioning(float & total_height, float & total_width) const
{
  Ogre::Real effective_char_height = char_height_ * effective_char_height_factor;

  total_height = effective_char_height;
  total_width = 0.0f;
  float current_width = 0.0f;
  for (auto & character : caption_) {
    if (character == '\n') {
      total_height += effective_char_height + line_spacing_;
      total_width = current_width > total_width ? current_width : total_width;
      current_width = 0;
    } else if (character == ' ') {
      current_width += space_width_;
    } else {
      current_width += font_->getGlyphAspectRatio(character) * effective_char_height;
    }
  }
  total_width = current_width > total_width ? current_width : total_width;
}

float MovableText::getVerticalStartFromVerticalAlignment(float total_height) const
{
  switch (vertical_alignment_) {
    case V_ABOVE:
      return total_height;
    case V_CENTER:
      return 0.5f * total_height;
    case V_BELOW:
      return 0.0f;
    default:
      throw std::runtime_error("unexpected vertical alignment");
  }
}

float MovableText::getLineStartFromHorizontalAlignment(float total_width) const
{
  switch (horizontal_alignment_) {
    case H_LEFT:
      return 0.0f;
    case H_CENTER:
      return -0.5f * total_width;
    default:
      throw std::runtime_error("unexpected horizontal alignment");
  }
}

void MovableText::fillVertexBuffer(
  Ogre::HardwareVertexBufferSharedPtr & position_and_texture_buffer, float top, float starting_left)
{
  Ogre::Real effective_char_height = char_height_ * effective_char_height_factor;

  auto hardware_buffer =
    static_cast<float *>(position_and_texture_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  auto buffer = TextBuffer(hardware_buffer);
  buffer.left_ = starting_left;
  buffer.top_ = top;

  for (auto & character : caption_) {
    if (character == '\n') {
      buffer.left_ = starting_left;
      buffer.top_ -= effective_char_height + line_spacing_;
      continue;
    }

    if (character == ' ') {
      buffer.left_ += space_width_;
      continue;
    }

    Ogre::Real char_aspect_ratio = font_->getGlyphAspectRatio(character);
    buffer.text_coords_ = font_->getGlyphTexCoords(character);
    float char_width = char_aspect_ratio * char_height_;

    buffer.addTopLeft();
    buffer.addBottomLeft(char_height_);
    buffer.addTopRight(char_width);

    buffer.addTopRight(char_width);
    buffer.addBottomLeft(char_height_);
    buffer.addBottomRight(char_width, char_height_);

    buffer.left_ += char_aspect_ratio * effective_char_height;
  }

  position_and_texture_buffer->unlock();

  mBox = Ogre::AxisAlignedBox(buffer.min_, buffer.max_);
  radius_ = Ogre::Math::Sqrt(buffer.max_squared_radius_);
}

void MovableText::updateColors()
{
  assert(font_);
  assert(material_);

  Ogre::RGBA color = color_.getAsBYTE();
  fillColorBuffer(color);
  needs_color_update_ = false;
}

void MovableText::fillColorBuffer(Ogre::RGBA color) const
{
  Ogre::HardwareVertexBufferSharedPtr hardware_buffer =
    mRenderOp.vertexData->vertexBufferBinding->getBuffer(COLOUR_BINDING);
  auto color_buffer = static_cast<Ogre::RGBA *>(
    hardware_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  for (int i = 0; i < static_cast<int>(mRenderOp.vertexData->vertexCount); ++i) {
    *color_buffer++ = color;
  }
  hardware_buffer->unlock();
}

const Ogre::Quaternion & MovableText::getWorldOrientation() const
{
  assert(mCamera);
  return const_cast<Ogre::Quaternion &>(mCamera->getDerivedOrientation());
}

void MovableText::visitRenderables(Ogre::Renderable::Visitor * visitor, bool debug_renderables)
{
  visitor->visit(this, 0, debug_renderables);
}

const Ogre::Vector3 & MovableText::getWorldPosition() const
{
  assert(mParentNode);
  return mParentNode->_getDerivedPosition();
}

void MovableText::getWorldTransforms(Ogre::Matrix4 * xform) const
{
  if (this->isVisible() && mCamera) {
    Ogre::Matrix3 rot3x3, scale3x3 = Ogre::Matrix3::IDENTITY;

    mCamera->getDerivedOrientation().ToRotationMatrix(rot3x3);

    Ogre::Vector3 parent_position = mParentNode->_getDerivedPosition() + global_translation_;
    parent_position += rot3x3 * local_translation_;

    scale3x3[0][0] = mParentNode->_getDerivedScale().x / 2;
    scale3x3[1][1] = mParentNode->_getDerivedScale().y / 2;
    scale3x3[2][2] = mParentNode->_getDerivedScale().z / 2;

    *xform = (rot3x3 * scale3x3);
    xform->setTrans(parent_position);
  }
}

void MovableText::getRenderOperation(Ogre::RenderOperation & op)
{
  if (isVisible()) {
    update();
    op = mRenderOp;
  }
}

void MovableText::update()
{
  if (needs_update_) {
    setupGeometry();
  }
  if (needs_color_update_) {
    updateColors();
  }
}

void MovableText::_notifyCurrentCamera(Ogre::Camera * camera)
{
  mCamera = camera;
}

void MovableText::_updateRenderQueue(Ogre::RenderQueue * queue)
{
  if (isVisible()) {
    update();
    queue->addRenderable(this, mRenderQueueID, OGRE_RENDERABLE_DEFAULT_PRIORITY);
  }
}

}  // namespace rviz_rendering
