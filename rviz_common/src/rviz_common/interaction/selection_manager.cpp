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
#include <string>
#include <utility>
#include <vector>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <OgreCamera.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreRectangle2D.h>
#include <OgreRenderTexture.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include <QTimer>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_rendering/custom_parameter_indices.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rviz_common/logging.hpp"

#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/handler_manager_iface.hpp"
#include "rviz_common/interaction/selection_renderer.hpp"


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
  depth_pixel_box_.data = nullptr;
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
  depth_pixel_box_.data = nullptr;

  QTimer * timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(updateProperties()));
  timer->start(200);
}

SelectionManager::~SelectionManager()
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  setSelection(M_Picked());

  highlight_node_->getParentSceneNode()->removeAndDestroyChild(highlight_node_);
  delete highlight_rectangle_;

  for (auto & pixel_box : pixel_boxes_) {
    delete[] reinterpret_cast<uint8_t *>(pixel_box.data);
  }
  delete[] reinterpret_cast<uint8_t *>(depth_pixel_box_.data);

  delete property_model_;
}

void SelectionManager::setDebugMode(bool debug)
{
  renderer_->setDebugMode(debug);
}

void SelectionManager::initialize()
{
  renderer_->initialize();
  // Create our render textures
  setTextureSize(1);

  // Create our highlight rectangle
  Ogre::SceneManager * scene_manager = context_->getSceneManager();
  highlight_node_ = scene_manager->getRootSceneNode()->createChildSceneNode();

  std::string name("SelectionRect");
  static int count = 0;
  name += std::to_string(count++);
  highlight_rectangle_ = new Ogre::Rectangle2D(true);

  static const uint32_t texture_data[1] = {0xffff0080};
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.reset(new Ogre::MemoryDataStream(
      reinterpret_cast<void *>(const_cast<uint32_t *>(&texture_data[0])),
      4
    ));

  Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().loadRawData(
    name + "Texture",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    pixel_stream,
    1,
    1,
    Ogre::PF_R8G8B8A8,
    Ogre::TEX_TYPE_2D,
    0
    );

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
    name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setLightingEnabled(false);
  // material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
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

  handler_manager_ = context_->getHandlerManager();
}


bool SelectionManager::get3DPoint(
  Ogre::Viewport * viewport,
  int x,
  int y,
  Ogre::Vector3 & result_point)
{
  std::vector<Ogre::Vector3> result_points_temp;
  bool success = get3DPatch(viewport, x, y, 1, 1, true, result_points_temp);
  if (result_points_temp.empty()) {
    // return result_point unmodified if get point fails.
    return false;
  }
  result_point = result_points_temp[0];

  return success;
}


bool SelectionManager::getPatchDepthImage(
  Ogre::Viewport * viewport, int x, int y, unsigned width,
  unsigned height, std::vector<float> & depth_vector)
{
  unsigned int num_pixels = width * height;
  depth_vector.reserve(num_pixels);

  setDepthTextureSize(width, height);

  for (auto handler : handler_manager_->handlers_) {
    handler.second.lock()->preRenderPass(0);
  }

  if (render(
      SelectionRectangle(viewport, x, y, x + width, y + height),
      RenderTexture(
        depth_render_texture_, Dimensions(depth_texture_width_, depth_texture_height_), "Depth"),
      depth_pixel_box_))
  {
    auto data_ptr = reinterpret_cast<uint8_t *>(depth_pixel_box_.data);

    for (uint32_t pixel = 0; pixel < num_pixels; ++pixel) {
      uint8_t a = data_ptr[4 * pixel];
      uint8_t b = data_ptr[4 * pixel + 1];
      uint8_t c = data_ptr[4 * pixel + 2];

      int int_depth = (c << 16) | (b << 8) | a;
      float normalized_depth = (static_cast<float>(int_depth)) / static_cast<float>(0xffffff);
      depth_vector.push_back(normalized_depth * camera_->getFarClipDistance());
    }
  } else {
    RVIZ_COMMON_LOG_WARNING("Failed to render depth patch\n");
    return false;
  }

  for (auto handler : handler_manager_->handlers_) {
    handler.second.lock()->postRenderPass(0);
  }

  return true;
}


bool SelectionManager::get3DPatch(
  Ogre::Viewport * viewport,
  int x,
  int y,
  unsigned int width,
  unsigned int height,
  bool skip_missing,
  std::vector<Ogre::Vector3> & result_points)
{
  auto handler_lock = handler_manager_->lock();

  std::vector<float> depth_vector;

  if (!getPatchDepthImage(viewport, x, y, width, height, depth_vector)) {
    return false;
  }

  unsigned int pixel_counter = 0;
  Ogre::Matrix4 projection = camera_->getProjectionMatrix();
  float depth;

  for (unsigned int y_iter = 0; y_iter < height; ++y_iter) {
    for (unsigned int x_iter = 0; x_iter < width; ++x_iter) {
      depth = depth_vector[pixel_counter];

      // Deal with missing or invalid points
      if ( ( depth > camera_->getFarClipDistance()) || ( depth == 0)) {
        ++pixel_counter;
        if (!skip_missing) {
          result_points.push_back(Ogre::Vector3(NAN, NAN, NAN));
        }
        continue;
      }


      Ogre::Vector3 result_point;
      // We want to shoot rays through the center of pixels, not the corners,
      // so add .5 pixels to the x and y coordinate to get to the center
      // instead of the top left of the pixel.
      Ogre::Real screenx = static_cast<float>(x_iter + .5) / static_cast<float>(width);
      Ogre::Real screeny = static_cast<float>(y_iter + .5) / static_cast<float>(height);
      if (projection[3][3] == 0.0) {  // If this is a perspective projection
        // get world-space ray from camera & mouse coord
        Ogre::Ray vp_ray = camera_->getCameraToViewportRay(screenx, screeny);

        // transform ray direction back into camera coords
        Ogre::Vector3 dir_cam = camera_->getDerivedOrientation().Inverse() * vp_ray.getDirection();

        // normalize, so dir_cam.z == -depth
        dir_cam = dir_cam / dir_cam.z * depth * -1;

        // compute 3d point from camera origin and direction*/
        result_point = camera_->getDerivedPosition() + camera_->getDerivedOrientation() * dir_cam;
      } else { // else this must be an orthographic projection.
               // For orthographic projection, getCameraToViewportRay() does
               // the right thing for us, and the above math does not work.
        Ogre::Ray ray;
        camera_->getCameraToViewportRay(screenx, screeny, &ray);

        result_point = ray.getPoint(depth);
      }

      result_points.push_back(result_point);
      ++pixel_counter;
    }
  }

  return !result_points.empty();
}


void SelectionManager::setDepthTextureSize(unsigned width, unsigned height)
{
  // Cap and store requested texture size
  // It's probably an error if an invalid size is requested.
  if (width > 1024) {
    width = 1024;
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "SelectionManager::setDepthTextureSize invalid width requested. "
      "Max Width: 1024 -- Width requested: " << width << ".  Capping Width at 1024.");
  }

  if (depth_texture_width_ != width) {
    depth_texture_width_ = width;
  }

  if (height > 1024) {
    height = 1024;
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "SelectionManager::setDepthTextureSize invalid height requested. "
      "Max Height: 1024 -- Height requested: " << width << ".  Capping Height at 1024.");
  }

  if (depth_texture_height_ != height) {
    depth_texture_height_ = height;
  }

  if (!depth_render_texture_.get() || depth_render_texture_->getWidth() != width ||
    depth_render_texture_->getHeight() != height)
  {
    std::string tex_name = "DepthTexture";
    if (depth_render_texture_.get()) {
      tex_name = depth_render_texture_->getName();

      // destroy old
      Ogre::TextureManager::getSingleton().remove(
        tex_name,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    }

    depth_render_texture_ =
      Ogre::TextureManager::getSingleton().createManual(tex_name,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, depth_texture_width_, depth_texture_height_, 0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);

    depth_render_texture_->getBuffer()->getRenderTarget()->setAutoUpdated(false);
  }
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
      render_texture = Ogre::TextureManager::getSingleton().createManual(tex_name,
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
    setHighlightRect(highlight_.viewport, highlight_.x1, highlight_.y1, highlight_.x2,
      highlight_.y2);

#if 0
    M_Picked results;
    highlight_node_->setVisible(false);
    pick(highlight_.viewport, highlight_.x1, highlight_.y1, highlight_.x2, highlight_.y2, results);
    highlight_node_->setVisible(true);
#endif
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
  auto w = box.getWidth();
  auto h = box.getHeight();

  pixel_buffer_.clear();
  pixel_buffer_.reserve(w * h);

  for (uint32_t y = 0; y < h; ++y) {
    for (uint32_t x = 0; x < w; ++x) {
      uint32_t pos = (x + y * w) * 4;

      uint32_t pix_val = *reinterpret_cast<uint32_t *>(reinterpret_cast<uint8_t *>(box.data) + pos);
      uint32_t handle = colorToHandle(box.format, pix_val);

      pixel_buffer_.push_back(handle);
    }
  }
}

void SelectionManager::renderAndUnpack(
  const SelectionRectangle & selection_rectangle, uint32_t pass)
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
  if (render(selection_rectangle, tex, pixel_boxes_[pass])) {
    unpackColors(pixel_boxes_[pass]);
  }
}

bool SelectionManager::render(
  const SelectionRectangle & selection_rectangle,
  const RenderTexture & render_texture,
  Ogre::PixelBox & dst_box)
{
  return renderer_->render(camera_, selection_rectangle, render_texture, dst_box);
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
  CollObjectHandle handle, const Ogre::ColourValue & color,
  Ogre::SceneNode * node)
{
  if (!node) {
    return;
  }
  // Loop over all objects attached to this node.
  auto obj_it = node->getAttachedObjectIterator();
  while (obj_it.hasMoreElements()) {
    auto obj = obj_it.getNext();
    setPickData(handle, color, obj);
  }
  // Loop over and recurse into all child nodes.
  auto child_it = node->getChildIterator();
  while (child_it.hasMoreElements()) {
    auto child = dynamic_cast<Ogre::SceneNode *>(child_it.getNext());
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
  CollObjectHandle handle, const Ogre::ColourValue & color,
  Ogre::MovableObject * object)
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

void
SelectionManager::highlight(rviz_rendering::RenderWindow * window, int x1, int y1, int x2, int y2)
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
  M_Picked & results,
  bool single_render_pass)
{
  Ogre::Viewport * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(window);
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);
  // TODO(anhosi) use defered to lock at once with selection_mutex_
  auto handler_lock = handler_manager_->lock();


  bool need_additional_render = false;

  V_CollObject handles_by_pixel;
  S_CollObject need_additional;

  auto rectangle = SelectionRectangle(viewport, x1, y1, x2, y2);
  // First render is special... does the initial object picking, determines
  // which objects have been selected.
  // After that, individual handlers can specify that they need additional
  // renders (max # defined in kNumRenderTextures_).
  {
    for (const auto & handler : handler_manager_->handlers_) {
      handler.second.lock()->preRenderPass(0);
    }

    renderAndUnpack(rectangle, 0);

    for (const auto & handler : handler_manager_->handlers_) {
      handler.second.lock()->postRenderPass(0);
    }

    handles_by_pixel.reserve(pixel_buffer_.size());
    for (const auto & handle : pixel_buffer_) {
      handles_by_pixel.push_back(handle);

      if (handle == 0) {
        continue;
      }

      auto handler = handler_manager_->getHandler(handle);

      if (handler) {
        std::pair<M_Picked::iterator,
          bool> insert_result = results.insert(std::make_pair(handle, Picked(handle)));
        if (insert_result.second) {
          if (handler->needsAdditionalRenderPass(1) && !single_render_pass) {
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

    renderAndUnpack(rectangle, pass);

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

      if (handler_manager_->getHandler(handle)->needsAdditionalRenderPass(pass +
        1)) {
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

}  // namespace interaction
}  // namespace rviz_common
