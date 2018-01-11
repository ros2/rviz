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

  unsigned int vertexCount = 0;

  // count letters to determine how many vertices are needed
  std::string::iterator i = mCaption.begin();
  std::string::iterator iend = mCaption.end();
  for (; i != iend; ++i) {
    if ((*i != ' ') && (*i != '\n')) {
      vertexCount += 6;
    }
  }

  if (mRenderOp.vertexData) {
    delete mRenderOp.vertexData;
    mRenderOp.vertexData = NULL;
    mUpdateColors = true;
  }

  if (mCaption.empty()) {
    return;
  }

  if (!mRenderOp.vertexData) {
    mRenderOp.vertexData = new VertexData();
  }

  mRenderOp.indexData = 0;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = vertexCount;
  mRenderOp.operationType = RenderOperation::OT_TRIANGLE_LIST;
  mRenderOp.useIndexes = false;

  VertexDeclaration * decl = mRenderOp.vertexData->vertexDeclaration;
  VertexBufferBinding * bind = mRenderOp.vertexData->vertexBufferBinding;
  size_t offset = 0;

  // create/bind positions/tex.ccord. buffer
  if (!decl->findElementBySemantic(VES_POSITION)) {
    decl->addElement(POS_TEX_BINDING, offset, VET_FLOAT3, VES_POSITION);
  }

  offset += VertexElement::getTypeSize(VET_FLOAT3);

  if (!decl->findElementBySemantic(VES_TEXTURE_COORDINATES)) {
    decl->addElement(POS_TEX_BINDING, offset, Ogre::VET_FLOAT2,
      Ogre::VES_TEXTURE_COORDINATES, 0);
  }

  HardwareVertexBufferSharedPtr ptbuf =
    HardwareBufferManager::getSingleton().createVertexBuffer(
    decl->getVertexSize(POS_TEX_BINDING),
    mRenderOp.vertexData->vertexCount,
    HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);
  bind->setBinding(POS_TEX_BINDING, ptbuf);

  // Colours - store these in a separate buffer because they change less often
  if (!decl->findElementBySemantic(VES_DIFFUSE)) {
    decl->addElement(COLOUR_BINDING, 0, VET_COLOUR, VES_DIFFUSE);
  }

  HardwareVertexBufferSharedPtr cbuf =
    HardwareBufferManager::getSingleton().createVertexBuffer(
    decl->getVertexSize(COLOUR_BINDING),
    mRenderOp.vertexData->vertexCount,
    HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);
  bind->setBinding(COLOUR_BINDING, cbuf);

  float * pPCBuff =
    static_cast<float *>(ptbuf->lock(HardwareBuffer::HBL_DISCARD));

  Real spaceWidth = mSpaceWidth;
  // Derive space width from a capital A
  if (spaceWidth == 0) {
    spaceWidth = mpFont->getGlyphAspectRatio('A') * mCharHeight * 2.0f;
  }

  float total_height = mCharHeight * 2.0f;
  float total_width = 0.0f;
  float current_width = 0.0f;
  for (auto & iterator : mCaption) {
    if (iterator == '\n') {
      total_height += mCharHeight * 2.0f + mLineSpacing;
      total_width = current_width > total_width ? current_width : total_width;
    } else if(iterator == ' ') {
      current_width += spaceWidth;
    } else {
      current_width += mpFont->getGlyphAspectRatio(iterator) * mCharHeight * 2.0f;
    }
  }
  total_width = current_width > total_width ? current_width : total_width;

  float top = 0.0f;
  switch (mVerticalAlignment) {
    case MovableText::V_ABOVE:
      top = total_height;
      break;
    case MovableText::V_CENTER:
      top = 0.5f * total_height;
      break;
    case MovableText::V_BELOW:
      top = 0.0f;
      break;
  }

  float starting_left = 0.0f;
  switch (mHorizontalAlignment) {
    case MovableText::H_LEFT:
      starting_left = 0.0f;
      break;
    case MovableText::H_CENTER:
      starting_left = -total_width / 2.0f;
      break;
  }


  auto buffer = TextBuffer(pPCBuff);
  buffer.left_ = starting_left;
  buffer.top_ = top;

  for (i = mCaption.begin(); i != iend; ++i) {
    if (*i == '\n') {
      buffer.left_ = starting_left;
      buffer.top_ -= mCharHeight * 2.0f + mLineSpacing;
      continue;
    }

    if (*i == ' ') {
      // Just leave a gap, no triangles
      buffer.left_ += spaceWidth;
      continue;
    }

    Real char_aspect_ratio = mpFont->getGlyphAspectRatio(*i);
    buffer.text_coords_ = mpFont->getGlyphTexCoords(*i);

    buffer.addTopLeft();
    buffer.addBottomLeft(mCharHeight);
    buffer.addTopRight(char_aspect_ratio * mCharHeight);

    buffer.addTopRight(char_aspect_ratio * mCharHeight);
    buffer.addBottomLeft(mCharHeight);
    buffer.addBottomRight(char_aspect_ratio * mCharHeight, mCharHeight);

    buffer.left_ += char_aspect_ratio * mCharHeight * 2.0;
  }

  // Unlock vertex buffer
  ptbuf->unlock();

  // update AABB/Sphere radius
  mAABB = Ogre::AxisAlignedBox(buffer.min_, buffer.max_);
  mRadius = Ogre::Math::Sqrt(buffer.max_squared_radius_);

  if (mUpdateColors) {
    this->_updateColors();
  }

  mNeedUpdate = false;
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
