/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz_common/load_resource.hpp"

#include <string>

#include <QFile>  // NOLINT: cpplint cannot handle the include order here
#include <QPainter>  // NOLINT: cpplint cannot handle the include order here
#include <QPixmapCache>  // NOLINT: cpplint cannot handle the include order here

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "resource_retriever/retriever.hpp"

#include "rviz_common/logging.hpp"

namespace rviz_common
{

resource_retriever::MemoryResource getResource(const std::string & resource_path)
{
  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource res;
  try {
    res = retriever.get(resource_path);
  } catch (resource_retriever::Exception & e) {
    RVIZ_COMMON_LOG_DEBUG(e.what());
    return resource_retriever::MemoryResource();
  }

  return res;
}

QPixmap loadPixmap(QString url, bool fill_cache)
{
  QPixmap pixmap;

  // if it's in the cache, no need to locate
  if (QPixmapCache::find(url, &pixmap)) {
    return pixmap;
  }

  RVIZ_COMMON_LOG_DEBUG("Load pixmap at " + url.toStdString());

  auto image = getResource(url.toStdString());
  if (image.size != 0) {
    if (!pixmap.loadFromData(image.data.get(), static_cast<uint32_t>(image.size))) {
      RVIZ_COMMON_LOG_ERROR("Could not load pixmap " + url.toStdString());
    }
  }

  if (fill_cache) {
    QPixmapCache::insert(url, pixmap);
  }

  return pixmap;
}

QCursor getDefaultCursor(bool fill_cache)
{
  Q_UNUSED(fill_cache);
  return QCursor(Qt::ArrowCursor);
}

QCursor makeIconCursor(QString url, bool fill_cache)
{
  QPixmap icon = loadPixmap(url, fill_cache);
  if (icon.width() == 0 || icon.height() == 0) {
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "Could not load pixmap " << url.toStdString() << " -- "
        "using default cursor instead.");
    return getDefaultCursor();
  }
  QString cache_key = url + ".cursor";
  return makeIconCursor(icon, cache_key, fill_cache);
}

QCursor makeIconCursor(QPixmap icon, QString cache_key, bool fill_cache)
{
  // if it's in the cache, no need to locate
  QPixmap cursor_img;
  if (QPixmapCache::find(cache_key, &cursor_img) ) {
    return QCursor(cursor_img, 0, 0);
  }

  QPixmap base_cursor = loadPixmap("package://rviz_common/icons/cursor.svg", fill_cache);

  const int cursor_size = 32;

  cursor_img = QPixmap(cursor_size, cursor_size);
  cursor_img.fill(QColor(0, 0, 0, 0) );

  // copy base cursor & icon into one image
  QPainter painter(&cursor_img);

  int draw_x = 12;
  int draw_y = 16;

  // if the icon is too large, move it to the left
  if (draw_x + icon.width() > cursor_size) {
    draw_x = cursor_size - icon.width();
  }
  if (draw_y + icon.height() > cursor_size) {
    draw_y = cursor_size - icon.height();
  }

  painter.drawPixmap(0, 0, base_cursor);
  painter.drawPixmap(draw_x, draw_y, icon);

  if (fill_cache) {
    QPixmapCache::insert(cache_key, cursor_img);
  }

  return QCursor(cursor_img, 1, 1);
}

}  // namespace rviz_common
