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

#include "screenshot_dialog.hpp"

#include <utility>

#include <QCheckBox>  // NOLINT: cpplint is unable to handle the include order here
#include <QDateTime>  // NOLINT: cpplint is unable to handle the include order here
#include <QDialogButtonBox>  // NOLINT: cpplint is unable to handle the include order here
#include <QFileDialog>  // NOLINT: cpplint is unable to handle the include order here
#include <QHBoxLayout>  // NOLINT: cpplint is unable to handle the include order here
#include <QImageWriter>  // NOLINT: cpplint is unable to handle the include order here
#include <QLabel>  // NOLINT: cpplint is unable to handle the include order here
#include <QMessageBox>  // NOLINT: cpplint is unable to handle the include order here
// Included so we know that QPushButton inherits QAbstractButton
#include <QPushButton>  // NOLINT: cpplint is unable to handle the include order here
#include <QScreen>  // NOLINT: cpplint is unable to handle the include order here
#include <QTimer>  // NOLINT: cpplint is unable to handle the include order here
#include <QVBoxLayout>  // NOLINT: cpplint is unable to handle the include order here
#include <QWindow>  // NOLINT: cpplint is unable to handle the include order here

#include "scaled_image_widget.hpp"

namespace rviz_common
{

ScreenshotDialog::ScreenshotDialog(
  QWidget * main_window, QWidget * render_window,
  const QString & default_save_dir)
: QWidget(nullptr),    // This should be a top-level window to act like a dialog.
  main_window_(main_window),
  render_window_(render_window),
  save_full_window_(false),
  delay_timer_(new QTimer(this)),
  first_time_(true),
  default_save_dir_(default_save_dir)
{
  image_widget_ = new ScaledImageWidget(.5);

  takeScreenshotNow();

  QCheckBox * full_window_checkbox = new QCheckBox("Save entire rviz window");

  button_box_ = new QDialogButtonBox(
    QDialogButtonBox::Save |
    QDialogButtonBox::Retry |
    QDialogButtonBox::Cancel);

  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->addWidget(image_widget_, 100);
  main_layout->addWidget(new QLabel("Image will be saved at the original resolution."));
  main_layout->addWidget(full_window_checkbox);
  main_layout->addWidget(button_box_);

  setLayout(main_layout);

  connect(
    button_box_, SIGNAL(clicked(QAbstractButton*)), this,
    SLOT(onButtonClicked(QAbstractButton*)));
  connect(full_window_checkbox, SIGNAL(toggled(bool)), this, SLOT(setSaveFullWindow(bool)));
  connect(delay_timer_, SIGNAL(timeout()), this, SLOT(onTimeout()));
}

void ScreenshotDialog::showEvent(QShowEvent * event)
{
  if (first_time_) {
    QPoint center = main_window_->rect().center();
    move(center.x() - width() / 2, center.y() - height() / 2);

    first_time_ = false;
  }
  QWidget::showEvent(event);
}

void ScreenshotDialog::setSaveFullWindow(bool save_full_window)
{
  save_full_window_ = save_full_window;
  takeScreenshot();
}

void ScreenshotDialog::takeScreenshot()
{
  main_window_->raise();
  delay_timer_->start(100);
}

void ScreenshotDialog::onTimeout()
{
  delay_timer_->stop();
  takeScreenshotNow();
  raise();
  activateWindow();
}

void ScreenshotDialog::takeScreenshotNow()
{
  QScreen * screen = main_window_->windowHandle()->screen();
  if (save_full_window_) {
    screenshot_ = screen->grabWindow(main_window_->winId());
  } else {
    // There is a known issue in Qt where calling winId() on an OpenGL window
    // (OGRE being a wrapper for OpenGL) can result in rendering 'glitches" (See
    // https://doc.qt.io/qt-5/qopenglwidget.html). As a work-around, we can raise
    // the main window, causing the render window to re-render its graphics display.
    screenshot_ = screen->grabWindow(render_window_->winId());
    main_window_->raise();
  }
  image_widget_->setImage(screenshot_);
}

void ScreenshotDialog::onButtonClicked(QAbstractButton * clicked)
{
  if (clicked == button_box_->button(QDialogButtonBox::Save)) {
    save();
  } else if (clicked == button_box_->button(QDialogButtonBox::Retry)) {
    takeScreenshot();
  } else if (clicked == button_box_->button(QDialogButtonBox::Cancel)) {
    close();
  }
}

void ScreenshotDialog::save()
{
  QString default_save_file =
    default_save_dir_ +
    "/rviz_screenshot_" +
    QDateTime::currentDateTime().toString("yyyy_MM_dd-hh_mm_ss") +
    ".png";
  QString filename = QFileDialog::getSaveFileName(this, "Save image", default_save_file);
  if (filename != "") {
    QString with_slashes = QDir::fromNativeSeparators(filename);
    QString file_part = with_slashes.section('/', -1);
    default_save_dir_ = QDir::toNativeSeparators(with_slashes.section('/', 0, -2));
    Q_EMIT savedInDirectory(default_save_dir_);

    // If filename has no dot, like "image" or has a dot in the zeroth
    // position, like ".image", add ".png" to give a default file
    // format.
    if (file_part.lastIndexOf(".") <= 0) {
      filename += ".png";
    }
    QImageWriter writer(filename);
    if (writer.write(screenshot_.toImage())) {
      close();
    } else {
      QString error_message;
      if (writer.error() == QImageWriter::UnsupportedFormatError) {
        QString suffix = filename.section('.', -1);
        QString formats_string;
        QList<QByteArray> formats = QImageWriter::supportedImageFormats();
        formats_string = formats[0];
        for (int i = 1; i < formats.size(); i++) {
          formats_string += ", " + formats[i];
        }

        error_message =
          "File type '" + suffix + "' is not supported.\n" +
          "Supported image formats are: " + formats_string + "\n";
      } else {
        error_message = "Failed to write image to file " + filename;
      }

      QMessageBox::critical(this, "Error", error_message);
    }
  }
}

}  // namespace rviz_common
