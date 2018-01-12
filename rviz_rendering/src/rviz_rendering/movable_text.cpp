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

// TODO(wjwwood): revist style of this file.

// Adapted from: http://www.ogre3d.org/wiki/index.php/MovableText
//          now: http://www.ogre3d.org/tikiwiki/tiki-index.php?page=MovableText
// Original authors:
/*
 * File: MovableText.cpp
 *
 * description: This create create a billboarding object that display a text.
 *
 * @author  2003 by cTh see gavocanov@rambler.ru
 * @update  2006 by barraq see nospam@barraquand.com
 */

#include "rviz_rendering/movable_text.hpp"

#include <sstream>
#include <limits>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#else
# pragma warning(push)
# pragma warning(disable : 4251)
#endif

#include <OgreCamera.h>
#include <OgreFont.h>
#include <OgreFontManager.h>
#include <OgreHardwareBufferManager.h>
#include <OgreMaterialManager.h>
#include <OgreQuaternion.h>
#include <OgreRoot.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <string>
#include <algorithm>

#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

using namespace Ogre;  // NOLINT

#define POS_TEX_BINDING    0
#define COLOUR_BINDING     1

#define MATERIAL_GROUP "rviz_rendering"
#define EFFECTIVE_CHAR_HEIGHT_FACTOR 2

namespace rviz_rendering
{

MovableText::MovableText(
  const String & caption, const String & fontName, Real charHeight,
  const ColourValue & color)
: mFontName(fontName),
  mType("MovableText"),
  mCaption(caption),
  mHorizontalAlignment(H_LEFT),
  mVerticalAlignment(V_BELOW),
  mColor(color),
  mCharHeight(charHeight),
  mLineSpacing(0.01f),
  mSpaceWidth(0),
  mUpdateColors(true),
  mOnTop(false),
  mTimeUntilNextToggle(0),
  mGlobalTranslation(0.0f),
  mLocalTranslation(0.0f),
  mpCam(NULL),
  mpWin(NULL),
  mpFont(NULL)
{
  static int count = 0;
  std::stringstream ss;
  ss << "MovableText" << count++;
  mName = ss.str();

  mRenderOp.vertexData = NULL;
  this->setFontName(mFontName);
  // Set a reasonable default space width
  if (mSpaceWidth == 0) {
    mSpaceWidth = mpFont->getGlyphAspectRatio('A') * mCharHeight * EFFECTIVE_CHAR_HEIGHT_FACTOR;
  }
  this->_setupGeometry();
}

MovableText::~MovableText()
{
  if (mRenderOp.vertexData) {
    delete mRenderOp.vertexData;
  }
  // May cause crashing... check this and comment if it does
  if (mpMaterial) {
    MaterialManager::getSingletonPtr()->remove(mpMaterial->getName(), MATERIAL_GROUP);
  }
}

void MovableText::setFontName(const String & fontName)
{
  if (Ogre::MaterialManager::getSingletonPtr()->resourceExists(mName + "Material",
    MATERIAL_GROUP))
  {
    Ogre::MaterialManager::getSingleton().remove(mName + "Material", MATERIAL_GROUP);
  }

  if (mFontName != fontName || !mpMaterial || !mpFont) {
    mFontName = fontName;
    mpFont =
      reinterpret_cast<Font *>(FontManager::getSingleton().getByName(mFontName,
      MATERIAL_GROUP).get());
    if (!mpFont) {
      throw Exception(Exception::ERR_ITEM_NOT_FOUND, "Could not find font " +
              fontName, "MovableText::setFontName");
    }

    mpFont->load();
    if (mpMaterial) {
      MaterialManager::getSingletonPtr()->remove(mpMaterial->getName(), MATERIAL_GROUP);
      mpMaterial.reset();
    }

    mpMaterial = mpFont->getMaterial()->clone(mName + "Material");
    if (!mpMaterial->isLoaded()) {
      mpMaterial->load();
    }

    mpMaterial->setDepthCheckEnabled(!mOnTop);
    mpMaterial->setDepthBias(1.0, 1.0);
    mpMaterial->setDepthWriteEnabled(mOnTop);
    mpMaterial->setLightingEnabled(false);
    mNeedUpdate = true;
  }
}

void MovableText::setCaption(const String & caption)
{
  if (caption != mCaption) {
    mCaption = caption;
    mNeedUpdate = true;
  }
}

void MovableText::setColor(const ColourValue & color)
{
  if (color != mColor) {
    mColor = color;
    mUpdateColors = true;
  }
}

void MovableText::setCharacterHeight(Real height)
{
  if (height != mCharHeight) {
    mCharHeight = height;
    mNeedUpdate = true;
  }
}

void MovableText::setLineSpacing(Real height)
{
  if (height != mLineSpacing) {
    mLineSpacing = height;
    mNeedUpdate = true;
  }
}

void MovableText::setSpaceWidth(Real width)
{
  if (width != mSpaceWidth) {
    mSpaceWidth = width;
    mNeedUpdate = true;
  }
}

void MovableText::setTextAlignment(
  const HorizontalAlignment & horizontalAlignment,
  const VerticalAlignment & verticalAlignment)
{
  if (mHorizontalAlignment != horizontalAlignment) {
    mHorizontalAlignment = horizontalAlignment;
    mNeedUpdate = true;
  }
  if (mVerticalAlignment != verticalAlignment) {
    mVerticalAlignment = verticalAlignment;
    mNeedUpdate = true;
  }
}

void MovableText::setGlobalTranslation(Vector3 trans)
{
  mGlobalTranslation = trans;
}

void MovableText::setLocalTranslation(Vector3 trans)
{
  mLocalTranslation = trans;
}

void MovableText::showOnTop(bool show)
{
  if (mOnTop != show && mpMaterial) {
    mOnTop = show;
    mpMaterial->setDepthBias(1.0, 1.0);
    mpMaterial->setDepthCheckEnabled(!mOnTop);
    mpMaterial->setDepthWriteEnabled(mOnTop);
  }
}

struct TextBuffer {
  float * buffer_;
  Ogre::Vector3 min_;
  Ogre::Vector3 max_;
  Ogre::Real max_squared_radius_;
  float top_;
  float left_;
  Ogre::Font::UVRect text_coords_;

  TextBuffer(float* buffer)
    : buffer_(buffer),
    min_(9999999.0f),
    max_(-999999.0f),
    max_squared_radius_(0),
    top_(0),
    left_(0) {
  }

  void addTopLeft() {
    addPositionToBuffer(0, 0);
    addTextureToBuffer(text_coords_.left, text_coords_.top);
  }

  void addBottomLeft(float char_height) {
    addPositionToBuffer(0, 2.0f * char_height);
    addTextureToBuffer(text_coords_.left, text_coords_.bottom);
  }

  void addTopRight(float char_width) {
    addPositionToBuffer(2.0f * char_width, 0);
    addTextureToBuffer(text_coords_.right, text_coords_.top);
  }

  void addBottomRight(float char_widht, float char_height) {
    addPositionToBuffer(2.0f * char_widht, 2.0f * char_height);
    addTextureToBuffer(text_coords_.right, text_coords_.bottom);
  }

private :
  void addPositionToBuffer(float plus_left, float minus_top) {
    Ogre::Vector3 current_position = Ogre::Vector3(left_+plus_left, top_-minus_top, 0.0);
    *buffer_++ = current_position.x;
    *buffer_++ = current_position.y;
    *buffer_++ = current_position.z;

    min_.makeFloor(current_position);
    max_.makeCeil(current_position);
    max_squared_radius_ = std::max(max_squared_radius_, current_position.squaredLength());
  }

  void addTextureToBuffer(float texture_x, float texture_y) {
    *buffer_++ = texture_x;
    *buffer_++ = texture_y;
  }
};

void MovableText::_setupGeometry()
{
  assert(mpFont);
  assert(mpMaterial);

  if (mCaption.empty()) {
    return;
  }

  setupRenderOperation();
  HardwareVertexBufferSharedPtr position_and_texture_buffer = setupHardwareBuffers();

  float total_height;
  float total_width;
  calculateTotalDimensionsForPositioning(total_height, total_width);

  float starting_top = getVerticalStartFromVerticalAlignment(total_height);
  float starting_left = getLineStartFromHorizontalAlignment(total_width);

  fillVertexBuffer(position_and_texture_buffer, starting_top, starting_left);

  if (mUpdateColors) {
    this->_updateColors();
  }

  mNeedUpdate = false;
}

void
MovableText::calculateTotalDimensionsForPositioning(float & total_height, float & total_width) const
{
  Real effective_char_height = mCharHeight * EFFECTIVE_CHAR_HEIGHT_FACTOR;

  total_height = effective_char_height;
  total_width = 0.0f;
  float current_width = 0.0f;
  for (auto & character : mCaption) {
    if (character == '\n') {
      total_height += effective_char_height + mLineSpacing;
      total_width = current_width > total_width ? current_width : total_width;
    } else if(character == ' ') {
      current_width += mSpaceWidth;
    } else {
      current_width += mpFont->getGlyphAspectRatio(character) * effective_char_height;
    }
  }
  total_width = current_width > total_width ? current_width : total_width;
}

float MovableText::getVerticalStartFromVerticalAlignment(float total_height) const
{
  switch (mVerticalAlignment) {
    case V_ABOVE:
      return total_height;
    case V_CENTER:
      return 0.5f * total_height;
    case V_BELOW:
      return  0.0f;
    default:
      throw std::runtime_error("unexpected vertical alignment");
  }
}

float MovableText::getLineStartFromHorizontalAlignment(float total_width) const
{
  switch (mHorizontalAlignment) {
    case H_LEFT:
      return 0.0f;
    case H_CENTER:
      return -0.5f * total_width;
    default:
      throw std::runtime_error("unexpected horizontal alignment");
  }
}

void MovableText::fillVertexBuffer(HardwareVertexBufferSharedPtr & position_and_texture_buffer,
  float top, float starting_left)
{
  Real effective_char_height = mCharHeight * EFFECTIVE_CHAR_HEIGHT_FACTOR;

  float * hardware_buffer =
    static_cast<float *>(position_and_texture_buffer->lock(HardwareBuffer::HBL_DISCARD));

  auto buffer = TextBuffer(hardware_buffer);
  buffer.left_ = starting_left;
  buffer.top_ = top;

  for (auto & character : mCaption) {
    if (character == '\n') {
      buffer.left_ = starting_left;
      buffer.top_ -= effective_char_height + mLineSpacing;
      continue;
    }

    if (character == ' ') {
      buffer.left_ += mSpaceWidth;
      continue;
    }

    Real char_aspect_ratio = mpFont->getGlyphAspectRatio(character);
    buffer.text_coords_ = mpFont->getGlyphTexCoords(character);

    buffer.addTopLeft();
    buffer.addBottomLeft(mCharHeight);
    buffer.addTopRight(char_aspect_ratio * mCharHeight);

    buffer.addTopRight(char_aspect_ratio * mCharHeight);
    buffer.addBottomLeft(mCharHeight);
    buffer.addBottomRight(char_aspect_ratio * mCharHeight, mCharHeight);

    buffer.left_ += char_aspect_ratio * effective_char_height;
  }

  position_and_texture_buffer->unlock();

  mAABB = AxisAlignedBox(buffer.min_, buffer.max_);
  mRadius = Math::Sqrt(buffer.max_squared_radius_);
}

HardwareVertexBufferSharedPtr MovableText::setupHardwareBuffers() const
{
  VertexDeclaration * decl = mRenderOp.vertexData->vertexDeclaration;
  VertexBufferBinding * bind = mRenderOp.vertexData->vertexBufferBinding;
  size_t offset = 0;

  // create/bind positions/tex.ccord. buffer
  if (!decl->findElementBySemantic(VES_POSITION)) {
    decl->addElement(POS_TEX_BINDING, offset, VET_FLOAT3, VES_POSITION);
  }

  offset += VertexElement::getTypeSize(VET_FLOAT3);

  if (!decl->findElementBySemantic(VES_TEXTURE_COORDINATES)) {
    decl->addElement(POS_TEX_BINDING, offset, VET_FLOAT2,
      VES_TEXTURE_COORDINATES, 0);
  }

  HardwareVertexBufferSharedPtr position_and_texture_buffer =
    HardwareBufferManager::getSingleton().createVertexBuffer(
    decl->getVertexSize(POS_TEX_BINDING),
    mRenderOp.vertexData->vertexCount,
    HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);
  bind->setBinding(POS_TEX_BINDING, position_and_texture_buffer);

  // Colours - store these in a separate buffer because they change less often
  if (!decl->findElementBySemantic(VES_DIFFUSE)) {
    decl->addElement(COLOUR_BINDING, 0, VET_COLOUR, VES_DIFFUSE);
  }

  HardwareVertexBufferSharedPtr color_buffer =
    HardwareBufferManager::getSingleton().createVertexBuffer(
    decl->getVertexSize(COLOUR_BINDING),
    mRenderOp.vertexData->vertexCount,
    HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);
  bind->setBinding(COLOUR_BINDING, color_buffer);
  return position_and_texture_buffer;
}

void MovableText::setupRenderOperation()
{
  unsigned int vertex_count = calculateVertexCount();

  if (mRenderOp.vertexData) {
    delete mRenderOp.vertexData;
    mRenderOp.vertexData = nullptr;
    mUpdateColors = true;
  }

  if (!mRenderOp.vertexData) {
    mRenderOp.vertexData = new VertexData();
  }

  mRenderOp.indexData = nullptr;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = vertex_count;
  mRenderOp.operationType = RenderOperation::OT_TRIANGLE_LIST;
  mRenderOp.useIndexes = false;
}

unsigned int MovableText::calculateVertexCount() const
{
  unsigned int vertex_count = 0;
  for (auto & character : mCaption) {
    if ((character != ' ') && (character != '\n')) {
      vertex_count += 6;
    }
  }
  return vertex_count;
}

void MovableText::_updateColors(void)
{
  assert(mpFont);
  assert(mpMaterial);

  // Convert to system-specific
  RGBA color;
  Root::getSingleton().convertColourValue(mColor, &color);
  HardwareVertexBufferSharedPtr vbuf =
    mRenderOp.vertexData->vertexBufferBinding->getBuffer(COLOUR_BINDING);
  RGBA * pDest = static_cast<RGBA *>(vbuf->lock(HardwareBuffer::HBL_DISCARD));
  for (int i = 0; i < static_cast<int>(mRenderOp.vertexData->vertexCount); ++i) {
    *pDest++ = color;
  }
  vbuf->unlock();
  mUpdateColors = false;
}

const Quaternion & MovableText::getWorldOrientation(void) const
{
  assert(mpCam);
  return const_cast<Quaternion &>(mpCam->getDerivedOrientation());
}

#if ( (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6) || OGRE_VERSION_MAJOR >= 2 )
void MovableText::visitRenderables(Ogre::Renderable::Visitor * visitor, bool debugRenderables)
{
  (void) debugRenderables;
  visitor->visit(this, 0, false);
}
#endif

const Vector3 & MovableText::getWorldPosition(void) const
{
  assert(mParentNode);
  return mParentNode->_getDerivedPosition();
}

void MovableText::getWorldTransforms(Matrix4 * xform) const
{
  if (this->isVisible() && mpCam) {
    Matrix3 rot3x3, scale3x3 = Matrix3::IDENTITY;

    // store rotation in a matrix
    mpCam->getDerivedOrientation().ToRotationMatrix(rot3x3);

    // parent node position
    Vector3 ppos = mParentNode->_getDerivedPosition() + Vector3::UNIT_Y *
      mGlobalTranslation;
    ppos += rot3x3 * mLocalTranslation;

    // apply scale
    scale3x3[0][0] = mParentNode->_getDerivedScale().x / 2;
    scale3x3[1][1] = mParentNode->_getDerivedScale().y / 2;
    scale3x3[2][2] = mParentNode->_getDerivedScale().z / 2;

    // apply all transforms to xform
    *xform = (rot3x3 * scale3x3);
    xform->setTrans(ppos);
  }
}

void MovableText::getRenderOperation(RenderOperation & op)
{
  if (this->isVisible()) {
    if (mNeedUpdate) {
      this->_setupGeometry();
    }
    if (mUpdateColors) {
      this->_updateColors();
    }
    op = mRenderOp;
  }
}

void MovableText::_notifyCurrentCamera(Camera * cam)
{
  mpCam = cam;
}

void MovableText::_updateRenderQueue(RenderQueue * queue)
{
  if (this->isVisible()) {
    if (mNeedUpdate) {
      this->_setupGeometry();
    }
    if (mUpdateColors) {
      this->_updateColors();
    }

    queue->addRenderable(this, mRenderQueueID, OGRE_RENDERABLE_DEFAULT_PRIORITY);
    // queue->addRenderable(this, mRenderQueueID, RENDER_QUEUE_SKIES_LATE);
  }
}

}  // namespace rviz_rendering
