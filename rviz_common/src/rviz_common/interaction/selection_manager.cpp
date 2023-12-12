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

#include "rviz_common/interaction/selection_manager.hpp"

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <OgreCamera.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreRectangle2D.h>
#include <OgreRenderTexture.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

#include <QTimer>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_rendering/custom_parameter_indices.hpp"
#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/interaction/handler_manager_iface.hpp"
#include "rviz_common/interaction/selection_renderer.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_model.hpp"


namespace rviz_common
{
namespace interaction
{

using rviz_common::properties::Property;
using rviz_common::properties::PropertyTreeModel;

SelectionManager::SelectionManager(
  DisplayContext * context, std::shared_ptr<SelectionRenderer> renderer)
: context_(context),
  highlight_enabled_(false),
  property_model_(new PropertyTreeModel(new Property("root"))),
  renderer_(renderer)
{
  // TODO(Martin-Idel-SI): I would like to just use setUpSlots(), but timer needs a QThread,
  // which is not easily available in tests
  for (auto & pixel_box : pixel_boxes_) {
    pixel_box.data = nullptr;
  }
}

SelectionManager::SelectionManager(DisplayContext * context)
: context_(context),
  highlight_enabled_(false),
  property_model_(new PropertyTreeModel(new Property("root"))),
  renderer_(std::make_shared<rviz_common::interaction::SelectionRenderer>(context))
{
  setUpSlots();
}

void SelectionManager::setUpSlots()
{
  for (auto & pixel_box : pixel_boxes_) {
    pixel_box.data = nullptr;
  }

  auto timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(updateProperties()));
  timer->start(200);
}

SelectionManager::~SelectionManager()
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  setSelection(M_Picked());

  highlight_node_->getParentSceneNode()->removeAndDestroyChild(highlight_node_);
  delete highlight_rectangle_;
  context_->getSceneManager()->destroyCamera(camera_);

  for (auto & pixel_box : pixel_boxes_) {
    delete[] static_cast<uint8_t *>(pixel_box.data);
  }

  delete property_model_;

  handler_manager_->removeListener(this);
}

void SelectionManager::initialize()
{
  // Create our render textures
  setTextureSize(1);

  // Create our highlight rectangle
  auto scene_manager = context_->getSceneManager();
  highlight_node_ = scene_manager->getRootSceneNode()->createChildSceneNode();

  std::string name("SelectionRect");
  static int count = 0;
  name += std::to_string(count++);
  highlight_rectangle_ = new Ogre::Rectangle2D(true);

  static const uint32_t texture_data[1] = {0xffff0080};
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.reset(
    new Ogre::MemoryDataStream(
      reinterpret_cast<void *>(const_cast<uint32_t *>(&texture_data[0])), 4
  ));

  Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().loadRawData(
    name + "Texture",
    "rviz_rendering",
    pixel_stream,
    1,
    1,
    Ogre::PF_R8G8B8A8,
    Ogre::TEX_TYPE_2D,
    0
  );

  Ogre::MaterialPtr material =
    rviz_rendering::MaterialManager::createMaterialWithShadowsAndNoLighting(name);
  highlight_rectangle_->setMaterial(material);
  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  highlight_rectangle_->setBoundingBox(aabInf);
  highlight_rectangle_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setCullingMode(Ogre::CULL_NONE);

  Ogre::TextureUnitState * tex_unit =
    material->getTechnique(0)->getPass(0)->createTextureUnitState();
  tex_unit->setTextureName(tex->getName());
  tex_unit->setTextureFiltering(Ogre::TFO_NONE);

  highlight_node_->attachObject(highlight_rectangle_);

  // create picking camera
  camera_ = scene_manager->createCamera(name + "_camera");
  auto camera_node = scene_manager->getRootSceneNode()->createChildSceneNode();
  camera_node->attachObject(camera_);

  renderer_->initialize(camera_);

  handler_manager_ = context_->getHandlerManager();
  handler_manager_->addListener(this);
}

void SelectionManager::setTextureSize(unsigned size)
{
  if (size > 1024) {
    size = 1024;
  }

  texture_size_ = size;

  for (auto & render_texture : render_textures_) {
    // check if we need to change the texture size
    if (!render_texture.get() || render_texture->getWidth() != size) {
      std::string tex_name;
      if (render_texture.get()) {
        tex_name = render_texture->getName();

        // destroy old
        Ogre::TextureManager::getSingleton().remove(
          tex_name,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      } else {
        static int count = 0;
        tex_name = "SelectionTexture" + std::to_string(count++);
      }

      // create new texture
      render_texture = Ogre::TextureManager::getSingleton().createManual(
        tex_name,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, size, size, 0,
        Ogre::PF_R8G8B8A8, Ogre::TU_STATIC | Ogre::TU_RENDERTARGET);

      render_texture->getBuffer()->getRenderTarget()->setAutoUpdated(false);
    }
  }
}

void SelectionManager::update()
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  highlight_node_->setVisible(highlight_enabled_);

  if (highlight_enabled_) {
    setHighlightRect(
      highlight_.viewport, highlight_.x1, highlight_.y1, highlight_.x2, highlight_.y2);
  }
}

void SelectionManager::removeHighlight()
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  highlight_enabled_ = false;
}

void SelectionManager::setHighlightRect(Ogre::Viewport * viewport, int x1, int y1, int x2, int y2)
{
  float nx1 = (static_cast<float>(x1) / viewport->getActualWidth()) * 2 - 1;
  float nx2 = (static_cast<float>(x2) / viewport->getActualWidth()) * 2 - 1;
  float ny1 = -((static_cast<float>(y1) / viewport->getActualHeight()) * 2 - 1);
  float ny2 = -((static_cast<float>(y2) / viewport->getActualHeight()) * 2 - 1);

  nx1 = nx1 < -1 ? -1 : (nx1 > 1 ? 1 : nx1);
  ny1 = ny1 < -1 ? -1 : (ny1 > 1 ? 1 : ny1);
  nx2 = nx2 < -1 ? -1 : (nx2 > 1 ? 1 : nx2);
  ny2 = ny2 < -1 ? -1 : (ny2 > 1 ? 1 : ny2);

  highlight_rectangle_->setCorners(nx1, ny1, nx2, ny2);
}

void SelectionManager::unpackColors(const Ogre::PixelBox & box)
{
  uint32_t w = box.getWidth();
  uint32_t h = box.getHeight();

  pixel_buffer_.clear();
  pixel_buffer_.reserve(w * h);

  size_t size = Ogre::PixelUtil::getMemorySize(1, 1, 1, box.format);

  for (uint32_t y = 0; y < h; y++) {
    for (uint32_t x = 0; x < w; x++) {
      uint32_t pos = static_cast<uint32_t>((x + y * w) * size);
      uint32_t pix_val = 0;
      memcpy(
        reinterpret_cast<uint8_t *>(&pix_val),
        reinterpret_cast<uint8_t *>(box.data + pos),
        size);
      pixel_buffer_.push_back(colorToHandle(box.format, pix_val));
    }
  }
}

void SelectionManager::renderAndUnpack(
  rviz_rendering::RenderWindow * window,
  const SelectionRectangle & selection_rectangle,
  uint32_t pass)
{
  assert(pass < render_textures_.size());

  std::stringstream scheme;
  scheme << "Pick";
  if (pass > 0) {
    scheme << pass;
  }

  auto tex = RenderTexture(
    render_textures_[pass],
    Dimensions(texture_size_, texture_size_),
    scheme.str());

  render(window, selection_rectangle, tex, pixel_boxes_[pass]);
  unpackColors(pixel_boxes_[pass]);
}

void SelectionManager::render(
  rviz_rendering::RenderWindow * window,
  const SelectionRectangle & selection_rectangle,
  const RenderTexture & render_texture,
  Ogre::PixelBox & dst_box)
{
  auto handler_lock = handler_manager_->lock();
  renderer_->render(
    window, selection_rectangle,
    render_texture,
    handler_manager_->handlers(),
    dst_box);
}

PropertyTreeModel * SelectionManager::getPropertyModel()
{
  return property_model_;
}

Ogre::ColourValue SelectionManager::handleToColor(CollObjectHandle handle)
{
  float r = ((handle >> 16) & 0xff) / 255.0f;
  float g = ((handle >> 8) & 0xff) / 255.0f;
  float b = (handle & 0xff) / 255.0f;
  return Ogre::ColourValue(r, g, b, 1.0f);
}

void SelectionManager::setPickColor(const Ogre::ColourValue & color, Ogre::SceneNode * node)
{
  setPickData(colorToHandle(color), color, node);
}

void SelectionManager::setPickColor(const Ogre::ColourValue & color, Ogre::MovableObject * object)
{
  setPickData(colorToHandle(color), color, object);
}

void SelectionManager::setPickHandle(CollObjectHandle handle, Ogre::SceneNode * node)
{
  setPickData(handle, handleToColor(handle), node);
}

void SelectionManager::setPickHandle(CollObjectHandle handle, Ogre::MovableObject * object)
{
  setPickData(handle, handleToColor(handle), object);
}

void SelectionManager::setPickData(
  CollObjectHandle handle, const Ogre::ColourValue & color, Ogre::SceneNode * node)
{
  if (!node) {
    return;
  }
  // Loop over all objects attached to this node.
  auto objects = node->getAttachedObjects();
  for (const auto & object : objects) {
    setPickData(handle, color, object);
  }
  // Loop over and recurse into all child nodes.
  for (auto child_node : node->getChildren()) {
    auto child = dynamic_cast<Ogre::SceneNode *>(child_node);
    setPickData(handle, color, child);
  }
}

class PickColorSetter : public Ogre::Renderable::Visitor
{
public:
  PickColorSetter(CollObjectHandle handle, const Ogre::ColourValue & color)
  : color_vector_(color.r, color.g, color.b, 1.0), handle_(handle) {}

  void visit(Ogre::Renderable * rend, ushort lodIndex, bool isDebug, Ogre::Any * pAny = 0) override
  {
    Q_UNUSED(lodIndex);
    Q_UNUSED(isDebug);
    Q_UNUSED(pAny);
    rend->setCustomParameter(RVIZ_RENDERING_PICK_COLOR_PARAMETER, color_vector_);
    rend->getUserObjectBindings().setUserAny("pick_handle", Ogre::Any(handle_));
  }

  Ogre::Vector4 color_vector_;
  CollObjectHandle handle_;
};

void SelectionManager::setPickData(
  CollObjectHandle handle, const Ogre::ColourValue & color, Ogre::MovableObject * object)
{
  PickColorSetter visitor(handle, color);
  object->visitRenderables(&visitor);
  object->getUserObjectBindings().setUserAny("pick_handle", Ogre::Any(handle));
}

void SelectionManager::removeSelection(const M_Picked & objs)
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  for (const auto & obj : objs) {
    removeSelectedObject(obj.second);
  }

  selectionRemoved(objs);
}

const M_Picked & SelectionManager::getSelection() const
{
  return selection_;
}

void SelectionManager::addSelection(const M_Picked & objs)
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  M_Picked added;
  for (const auto & obj : objs) {
    auto ppb = addSelectedObject(obj.second);
    if (ppb.second) {
      added.insert(std::make_pair(obj.first, ppb.first));
    }
  }

  selectionAdded(added);
}

void SelectionManager::setSelection(const M_Picked & objs)
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  M_Picked original(selection_.begin(), selection_.end());

  removeSelection(original);
  addSelection(objs);
}

std::pair<Picked, bool> SelectionManager::addSelectedObject(const Picked & obj)
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  std::pair<M_Picked::iterator, bool> pib = selection_.insert(std::make_pair(obj.handle, obj));

  auto handler = handler_manager_->getHandler(obj.handle);

  if (pib.second) {
    handler->onSelect(obj);
    return std::make_pair(obj, true);
  } else {
    Picked & cur = pib.first->second;
    Picked added(cur.handle);

    for (const auto & extra_handle  : obj.extra_handles) {
      if (cur.extra_handles.insert(extra_handle).second) {
        added.extra_handles.insert(extra_handle);
      }
    }

    if (!added.extra_handles.empty()) {
      handler->onSelect(added);

      return std::make_pair(added, true);
    }
  }

  return std::make_pair(Picked(0), false);
}

void SelectionManager::removeSelectedObject(const Picked & obj)
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  auto sel_it = selection_.find(obj.handle);
  if (sel_it != selection_.end()) {
    for (const auto & extra_handle : obj.extra_handles) {
      sel_it->second.extra_handles.erase(extra_handle);
    }

    if (sel_it->second.extra_handles.empty()) {
      selection_.erase(sel_it);
    }
  }

  handler_manager_->getHandler(obj.handle)->onDeselect(obj);
}

void SelectionManager::focusOnSelection()
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  if (selection_.empty()) {
    return;
  }

  Ogre::AxisAlignedBox combined;

  for (const auto & selection_item : selection_) {
    const Picked & p = selection_item.second;

    auto handler = handler_manager_->getHandler(p.handle);

    auto aabbs = handler->getAABBs(p);

    for (const auto & aabb : aabbs) {
      combined.merge(aabb);
    }
  }

  if (!combined.isInfinite() && !combined.isNull()) {
    Ogre::Vector3 center = combined.getCenter();
    ViewController * controller = context_->getViewManager()->getCurrent();
    if (controller) {
      controller->lookAt(center);
    }
  }
}

void SelectionManager::selectionRemoved(const M_Picked & removed)
{
  for (const auto & removed_item : removed) {
    const Picked & picked = removed_item.second;
    auto handler = handler_manager_->getHandler(picked.handle);
    assert(handler);

    handler->destroyProperties(picked, property_model_->getRoot());
  }
}

void SelectionManager::selectionAdded(const M_Picked & added)
{
  for (const auto & added_item : added) {
    const Picked & picked = added_item.second;
    auto handler = handler_manager_->getHandler(picked.handle);
    assert(handler);

    handler->createProperties(picked, property_model_->getRoot());
  }
  property_model_->sort(0, Qt::AscendingOrder);
}

void SelectionManager::updateProperties()
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  for (const auto & selection_item : selection_) {
    handler_manager_->getHandler(selection_item.first)->updateProperties();
  }
}

void SelectionManager::highlight(
  rviz_rendering::RenderWindow * window, int x1, int y1, int x2, int y2)
{
  Ogre::Viewport * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(window);
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  highlight_enabled_ = true;

  highlight_.viewport = viewport;
  highlight_.x1 = x1;
  highlight_.y1 = y1;
  highlight_.x2 = x2;
  highlight_.y2 = y2;
}

void SelectionManager::select(
  rviz_rendering::RenderWindow * window, int x1, int y1, int x2, int y2, SelectType type)
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  highlight_enabled_ = false;
  highlight_node_->setVisible(false);

  M_Picked results;
  pick(window, x1, y1, x2, y2, results);

  if (type == Add) {
    addSelection(results);
  } else if (type == Remove) {
    removeSelection(results);
  } else if (type == Replace) {
    setSelection(results);
  }
}

void SelectionManager::pick(
  rviz_rendering::RenderWindow * window,
  int x1,
  int y1,
  int x2,
  int y2,
  M_Picked & results)
{
  auto handler_lock = handler_manager_->lock(std::defer_lock);
  std::lock(selection_mutex_, handler_lock);
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_, std::adopt_lock);

  bool need_additional_render = false;

  V_CollObject handles_by_pixel;
  S_CollObject need_additional;

  auto rectangle = SelectionRectangle(x1, y1, x2, y2);
  // First render is special... does the initial object picking, determines
  // which objects have been selected.
  // After that, individual handlers can specify that they need additional
  // renders (max # defined in kNumRenderTextures_).
  {
    renderAndUnpack(window, rectangle, 0);

    handles_by_pixel.reserve(pixel_buffer_.size());
    for (const auto & handle : pixel_buffer_) {
      handles_by_pixel.push_back(handle);

      if (handle == 0) {
        continue;
      }

      auto handler = handler_manager_->getHandler(handle);

      if (handler) {
        std::pair<M_Picked::iterator, bool> insert_result =
          results.insert(std::make_pair(handle, Picked(handle)));
        if (insert_result.second) {
          if (handler->needsAdditionalRenderPass(1)) {
            need_additional.insert(handle);
            need_additional_render = true;
          }
        } else {
          insert_result.first->second.pixel_count++;
        }
      }
    }
  }

  uint32_t pass = 1;

  V_uint64 extra_by_pixel;
  extra_by_pixel.resize(handles_by_pixel.size());
  while (need_additional_render && pass < render_textures_.size()) {
    for (const auto & handle : need_additional) {
      handler_manager_->getHandler(handle)->preRenderPass(pass);
    }

    renderAndUnpack(window, rectangle, pass);

    for (const auto & handle : need_additional) {
      handler_manager_->getHandler(handle)->postRenderPass(pass);
    }

    for (size_t i = 0; i != handles_by_pixel.size(); ++i) {
      if (pass == 1) {
        extra_by_pixel[i] = 0;
      }

      if (need_additional.find(handles_by_pixel[i]) != need_additional.end()) {
        auto extra_handle = pixel_buffer_[i];
        extra_by_pixel[i] |= extra_handle << (32 * (pass - 1));
      } else {
        extra_by_pixel[i] = 0;
      }
    }

    need_additional_render = false;
    need_additional.clear();
    for (const auto & result : results) {
      CollObjectHandle handle = result.first;

      if (handler_manager_->getHandler(handle)->needsAdditionalRenderPass(pass + 1)) {
        need_additional_render = true;
        need_additional.insert(handle);
      }
    }
  }

  for (size_t i = 0; i != handles_by_pixel.size(); ++i) {
    auto handle = handles_by_pixel[i];

    if (handle == 0) {
      continue;
    }

    auto picked_it = results.find(handle);
    if (picked_it == results.end()) {
      continue;
    }

    Picked & picked = picked_it->second;

    if (extra_by_pixel[i]) {
      picked.extra_handles.insert(extra_by_pixel[i]);
    }
  }
}

void SelectionManager::onHandlerRemoved(CollObjectHandle handle)
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  auto it = selection_.find(handle);
  if (it != selection_.end()) {
    selection_.erase(it);
  }
}

}  // namespace interaction
}  // namespace rviz_common
