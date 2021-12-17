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

#ifndef RVIZ_COMMON__FACTORY__PLUGINLIB_FACTORY_HPP_
#define RVIZ_COMMON__FACTORY__PLUGINLIB_FACTORY_HPP_

#include <string>
#include <vector>

#include <QHash>  // NOLINT
#include <QString>  // NOLINT
#include <QStringList>  // NOLINT

#ifndef Q_MOC_RUN
#include "pluginlib/class_loader.hpp"
#endif

#include "rviz_common/factory/factory.hpp"
#include "rviz_common/factory/class_id_recording_factory.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/logging.hpp"

namespace rviz_common
{

template<class Type>
class PluginlibFactory : public ClassIdRecordingFactory<Type>
{
private:
  struct BuiltInClassRecord
  {
    QString class_id_;
    QString package_;
    QString name_;
    QString description_;
    std::function<Type * ()> factory_function_;
  };

public:
  PluginlibFactory(const QString & package, const QString & base_class_type)
  {
    class_loader_ = new pluginlib::ClassLoader<Type>(
      package.toStdString(), base_class_type.toStdString());
  }

  virtual ~PluginlibFactory()
  {
    delete class_loader_;
  }

  std::vector<PluginInfo> getDeclaredPlugins() override
  {
    std::vector<PluginInfo> plugins;
    std::vector<std::string> std_ids = class_loader_->getDeclaredClasses();
    for (const auto & id : std_ids) {
      plugins.emplace_back(getPluginInfo(QString::fromStdString(id)));
    }
    typename QHash<QString, BuiltInClassRecord>::const_iterator iter;
    for (iter = built_ins_.begin(); iter != built_ins_.end(); ++iter) {
      plugins.emplace_back(getPluginInfo(iter->class_id_));
    }
    return plugins;
  }

  PluginInfo getPluginInfo(const QString & class_id) const override
  {
    PluginInfo info;
    typename QHash<QString, BuiltInClassRecord>::const_iterator iter = built_ins_.find(class_id);
    if (iter != built_ins_.end()) {
      info.id = iter->class_id_;
      info.name = iter->name_;
      info.package = iter->package_;
      info.description = iter->description_;
      info.icon = getIcon(info);
      return info;
    }
    auto class_id_std = class_id.toStdString();
    info.id = class_id;
    info.name = QString::fromStdString(class_loader_->getName(class_id_std));
    info.package = QString::fromStdString(class_loader_->getClassPackage(class_id_std));
    info.description = QString::fromStdString(class_loader_->getClassDescription(class_id_std));
    info.icon = getIcon(info);
    return info;
  }

  virtual QString getPluginManifestPath(const QString & class_id) const
  {
    typename QHash<QString, BuiltInClassRecord>::const_iterator iter = built_ins_.find(class_id);
    if (iter != built_ins_.end()) {
      return "";
    }
    return QString::fromStdString(class_loader_->getPluginManifestPath(class_id.toStdString()));
  }

  virtual void addBuiltInClass(
    const QString & package, const QString & name, const QString & description,
    std::function<Type * ()> factory_function)
  {
    BuiltInClassRecord record;
    record.class_id_ = package + "/" + name;
    record.package_ = package;
    record.name_ = name;
    record.description_ = description;
    record.factory_function_ = factory_function;
    built_ins_[record.class_id_] = record;
  }

protected:
  /// Instantiate and return a instance of a subclass of Type using our pluginlib::ClassLoader.
  /**
   * If makeRaw() returns nullptr and error_return is not nullptr,
   * *error_return will be set.
   * On success, *error_return will not be changed.
   *
   * \param class_id A string identifying the class uniquely among classes of
   *   its parent class, e.g. rviz::GridDisplay might be 'rviz/Grid'.
   * \param error_return If non-nullptr and there is an error, *error_return is
   *   set to a description of the problem.
   * \return A new instance of the class identified by class_id, or
   *   nullptr if there was an error.
   */
  Type * makeRaw(const QString & class_id, QString * error_return = nullptr) override
  {
    typename QHash<QString, BuiltInClassRecord>::const_iterator iter = built_ins_.find(class_id);
    if (iter != built_ins_.end()) {
      Type * instance = iter->factory_function_();
      if (instance == nullptr && error_return != nullptr) {
        *error_return = "Factory function for built-in class '" + class_id + "' returned nullptr.";
      }
      return instance;
    }
    try {
      return class_loader_->createUnmanagedInstance(class_id.toStdString());
    } catch (pluginlib::PluginlibException & ex) {
      RVIZ_COMMON_LOG_ERROR_STREAM(
        "PluginlibFactory: The plugin for class '" <<
          qPrintable(class_id) << "' failed to load. Error: " << ex.what());
      if (error_return) {
        *error_return = QString::fromStdString(ex.what());
      }
      return nullptr;
    }
  }

  virtual QIcon getIcon(const PluginInfo & info) const
  {
    auto default_icon_path = "package://rviz_common/icons/default_class_icon.png";

    if (info.package.isEmpty() || info.name.isEmpty()) {
      return loadPixmap(default_icon_path);
    }

    auto base_path = "package://" + info.package + "/icons/classes/" + info.name;
    QIcon icon = loadPixmap(base_path + ".svg");
    if (icon.isNull()) {
      icon = loadPixmap(base_path + ".png");
      if (icon.isNull()) {
        icon = loadPixmap(default_icon_path);
      }
    }
    return icon;
  }

private:
  pluginlib::ClassLoader<Type> * class_loader_;
  QHash<QString, BuiltInClassRecord> built_ins_;
};

}  // end namespace rviz_common

#endif  // RVIZ_COMMON__FACTORY__PLUGINLIB_FACTORY_HPP_
