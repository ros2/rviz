// Copyright (c) 2023, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "rviz_common/properties/regex_filter_property.hpp"

#include <QValidator>
#include <QLineEdit>
#include <QString>
#include <QToolTip>
#include <QWidget>

#include <regex>

#include "rviz_common/properties/string_property.hpp"

namespace rviz_common
{
namespace properties
{
RegexValidator::RegexValidator(QLineEdit * editor)
: QValidator(editor), editor_(editor)
{
}

QValidator::State RegexValidator::validate(QString & input, int & /*pos*/) const
{
  try {
    std::regex(input.toLocal8Bit().constData());
    editor_->setStyleSheet(QString());
    QToolTip::hideText();
    return QValidator::Acceptable;
  } catch (const std::regex_error & e) {
    editor_->setStyleSheet("background: #ffe4e4");
    QToolTip::showText(editor_->mapToGlobal(QPoint(0, 5)), tr(e.what()), editor_, QRect(), 5000);
    return QValidator::Intermediate;
  }
}

void RegexFilterProperty::onValueChanged()
{
  const auto & value = getString();
  if (value.isEmpty()) {
    regex_ = std::regex(default_);
    regex_str_ = default_;
  } else {
    try {
      regex_str_ = std::string(value.toLocal8Bit().constData());
      regex_.assign(regex_str_, std::regex_constants::optimize);
    } catch (const std::regex_error &) {
      regex_ = std::regex(default_);
      regex_str_ = default_;
    }
  }
}

RegexFilterProperty::RegexFilterProperty(
  const QString & name, const std::string regex,
  Property * parent)
: StringProperty(name, "", "regular expression", parent), default_(regex), regex_(regex),
  regex_str_(regex)
{
  QObject::connect(this, &RegexFilterProperty::changed, this, [this]() {onValueChanged();});
}

const std::regex & RegexFilterProperty::regex() const
{
  return regex_;
}

const std::string & RegexFilterProperty::regex_str() const
{
  return regex_str_;
}

QWidget * RegexFilterProperty::createEditor(QWidget * parent, const QStyleOptionViewItem & option)
{
  auto * editor = qobject_cast<QLineEdit *>(StringProperty::createEditor(parent, option));
  if (editor) {
    editor->setValidator(new RegexValidator(editor));
  }
  return editor;
}
}  // end namespace properties
}  // end namespace rviz_common
