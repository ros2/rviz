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

#ifndef RVIZ_COMMON__PROPERTIES__REGEX_FILTER_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__REGEX_FILTER_PROPERTY_HPP_

#include <regex>
#include <string>

#include <QValidator>  // NOLINT: cpplint is unable to handle the include order here
#include <QLineEdit>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <QToolTip>  // NOLINT: cpplint is unable to handle the include order here
#include <QWidget>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{
class RVIZ_COMMON_PUBLIC RegexValidator : public QValidator
{
public:
  explicit RegexValidator(QLineEdit * editor);

  QValidator::State validate(QString & input, int & /*pos*/) const override;

private:
  QLineEdit * editor_;
};

class RVIZ_COMMON_PUBLIC RegexFilterProperty : public StringProperty
{
public:
  RegexFilterProperty(const QString & name, const std::string regex, Property * parent);

  const std::regex & regex() const;
  const std::string & regex_str() const;

  QWidget * createEditor(QWidget * parent, const QStyleOptionViewItem & option) override;

private:
  std::string default_;
  std::regex regex_;
  std::string regex_str_;

  void onValueChanged();
};
}  // end namespace properties
}  // end namespace rviz_common
#endif  // RVIZ_COMMON__PROPERTIES__REGEX_FILTER_PROPERTY_HPP_
