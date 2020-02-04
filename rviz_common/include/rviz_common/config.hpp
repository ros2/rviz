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

#ifndef RVIZ_COMMON__CONFIG_HPP_
#define RVIZ_COMMON__CONFIG_HPP_

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>

#include <QMap>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <QVariant>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{

/// Configuration data storage class.
/**
 * The purpose of the Config class is to provide a flexible place to
 * store configuration data during saving and loading which is
 * independent of the particular storage format (like YAML or XML or
 * INI).
 * The data is stored in a tree structure, supporting both
 * named and numerically-indexed children.
 * Leaves in the tree store QVariants, with convenience functions for int,
 * float, QString, and bool types.
 *
 * Config instances are references to an internal "Node" class which
 * actually stores the data and the tree structure.
 * Nodes are reference-counted and deletion is handled automatically.
 * This makes it safe to hold a reference to a portion of a Config tree to
 * use later, because the internal Nodes beneath the saved reference
 * will not be destroyed when the root of the tree goes out of scope.
 *
 * Config objects can be used on their own for generic hierarchical
 * data storage, but they are intended to be used with reader and
 * writer classes.
 * Currently there is just YAML support, as that is all that RViz supports
 * right now.
 * Those classes are YamlConfigReader and YamlConfigWriter.
 *
 * Typical use for reading looks like this:
 *
 *   YamlConfigReader reader;
 *   Config config;
 *   reader.readFile(config, "my_file.yaml");
 *   if(!reader.error()) {
 *     int height, width;
 *     if(
 *       config.mapGetString("Height", &height) &&
 *       config.mapGetString("Width", &width))
 *     {
 *       resize(width, height);
 *     }
 *
 *     Config file_list_config = config.mapGetChild("Files");
 *     filenames_.clear();
 *     int num_files = file_list_config.listLength();
 *     for(size_t i = 0; i < num_files; i++) {
 *       filenames_.push_back(
 *         file_list_config.listChildAt(i).getValue().toString());
 *     }
 *   } else {
 *     printf("%s", qPrintable(reader.errorMessage()));
 *   }
 *
 * For writing, the same program might use this:
 *
 *   Config config;
 *   config.mapSetValue("Height", height());
 *   config.mapSetValue("Width", width());
 *   Config file_list_config = config.mapMakeChild("Files");
 *   for(int i = 0; i < filenames_.size(); i++) {
 *     file_list_config.listAppendNew().setValue(filenames_[i]);
 *   }
 *
 *   YamlConfigWriter writer;
 *   writer.writeFile(config, "my_file.yaml");
 *
 *   if(writer.error()) {
 *     printf("%s", qPrintable(writer.errorMessage()));
 *   }
 *
 * setType() can be used to set the type of a given node (Map, List,
 * Value, or Empty), but it is often unnecessary.
 * All functions which add or change data (mapSetValue(), mapMakeChild(),
 * setValue(), and listAppendNew()) internally call setType() to ensure the
 * node has the right type for the operation.
 * If setType() is called with the same type that the node already has,
 * nothing happens.
 * If it needs to change the type of the node, any data stored in the node is
 * destroyed (except for child nodes which are referenced by other existing
 * Config objects).
 */
class RVIZ_COMMON_PUBLIC Config
{
private:
  class Node;
  typedef std::shared_ptr<Node> NodePtr;

public:
  /// The default constructor creates an empty config object.
  Config();

  /// The copy constructor copies only the reference to the data, not the data itself.
  Config(const Config & source);

  /// The converting constructor, makes a Value type Config object with the given value.
  explicit Config(QVariant value);

  Config &
  operator=(const Config & source);

  /// Make this object a deep copy of the source.
  void
  copy(const Config & source);

  /// Possible types a Config Node can have are Map, List, Value, and Empty.
  /**
   * Invalid means the Config object does not point to a Node at all.
   *
   * Invalid Config objects are returned by data access functions when
   * the data does not exist, like listChildAt(7) on a list of length
   * 3, or mapGetChild("foo") on a Value Node.
   */
  enum Type {Map, List, Value, Empty, Invalid};

  /// Return the Type of the referenced Node, or Invalid if no Node is referenced.
  Type
  getType() const;

  /// Set the type of this Config Node.
  /**
   * If new_type is Invalid, this de-references the node and makes
   * the Config object invalid.
   * If the new type is different from the old type, this deletes the existing
   * data in the Node and changes the Node's type to new_type.
   * If this does not change the type of the Node, no data is deleted and
   * nothing is changed.
   *
   * If this Config is currently invalid and new_type is not Invalid, this will
   * create a new Node and reference it.
   */
  void
  setType(Type new_type);

  /// Returns true if the internal Node reference is valid, false if not.
  /**
   * Same as (getType() != Invalid).
   */
  bool
  isValid() const;

  /// Set a named child to the given value.
  /**
   * Since QVariant has constructors for int, float, bool, QString,
   * and other supported types, you can call mapSetValue() directly
   * with your data in most cases:
   *
   *   config.mapSetValue("Size", 13);
   *   config.mapSetValue("Name", "Humphrey");
   *
   * mapSetValue(key, value) is the same as mapMakeChild(key).setValue(value).
   *
   * This forces the referenced Node to have type Map.
   */
  void
  mapSetValue(const QString & key, QVariant value);

  /// Create a child node stored with the given key, and return the child.
  /**
   * This forces the referenced Node to have type Map.
   */
  Config
  mapMakeChild(const QString & key);

  /// Return a reference to the child if the Node is a Map which contains the given key.
  /**
   * If the reference is invalid or the Node has a different Type, return an
   * invalid Config.
   */
  Config
  mapGetChild(const QString & key) const;

  /// Convenience function for looking up a named value.
  /**
   * If a Value Node with the given key is a child of this Node, set
   * value_out to the given value and return true.
   *
   * If the Config is Invalid or the Node is not a Map, this returns an
   * Invalid Config.
   */
  bool
  mapGetValue(const QString & key, QVariant * value_out) const;

  /// Convenience function for looking up a named integer.
  /**
   * If a Value Node with the given key is a child of this Node, and
   * the Value is either an int or a string-ified int, set value_out
   * to the integer and return true.
   *
   * If the Config is invalid or the Node is not a Map, returns an
   * Invalid Config.
   */
  bool
  mapGetInt(const QString & key, int * value_out) const;

  /// Convenience function for looking up a named float.
  /**
   * If a Value Node with the given key is a child of this Node, and
   * the Value is either a float, a double or a string-ified float or
   * double, set value_out to the float and return true.
   *
   * If the Config is invalid or the Node is not a Map, returns an
   * Invalid Config.
   */
  bool
  mapGetFloat(const QString & key, float * value_out) const;

  /// Convenience function for looking up a named boolean.
  /**
   * If a Value Node with the given key is a child of this Node, and
   * the Value is either a bool or a string-ified bool, set value_out
   * to the bool and return true.
   *
   * If the Config is invalid or the Node is not a Map, returns an
   * Invalid Config.
   */
  bool
  mapGetBool(const QString & key, bool * value_out) const;

  /// Convenience function for looking up a named string.
  /**
   * If a Value Node with the given key is a child of this Node, and
   * the Value is a string, set value_out to the string and return
   * true.
   *
   * If the Config is invalid or the Node is not a Map, returns an
   * Invalid Config.
   */
  bool
  mapGetString(const QString & key, QString * value_out) const;

  /// Ensure this is a valid Config object, set the type to Value, then set the value.
  /// Set the value and the type to Value, if the Config object is valid.
  void
  setValue(const QVariant & value);

  /// Return the value, if this Config object is valid and is a Value type.
  /**
   * Otherwise it returns an invalid QVariant.
   */
  QVariant
  getValue() const;

  /// Return the length of the List in this Node, or 0 if this Node does not have type List.
  int
  listLength() const;

  /// Return the i'th child in the list, if the referenced Node has type List.
  /**
   * Returns an Invalid Config if the type is not List or if i is not a valid
   * index into it.
   */
  Config
  listChildAt(int i) const;

  /// Append a new empty Node to the list and return a reference, unless the Node is not a List.
  Config
  listAppendNew();

  /// Iterator class for looping over all entries in a Map type Config Node.
  /**
   * Typical usage:
   *
   *   Config config;
   *   display->save(config);  // Write display's data into config.
   *   for(Config::MapIterator iter = config.mapIterator(); iter.isValid(); iter.advance()) {
   *     QString key = iter.currentKey();
   *     Config child = iter.currentChild();
   *     printf(
   *       "key %s has value %s.\n",
   *       qPrintable(ke ), qPrintable(child.getValue().toString()));
   *   }
   *
   * Maps are stored in alphabetical order of their keys, and MapIterator uses
   * this same order.
   */
  class RVIZ_COMMON_PUBLIC MapIterator
  {
    // *INDENT-OFF*
  public:
    // *INDENT-ON*
    /// Advance iterator to next entry.
    void
    advance();

    /// Return true if the iterator currently points to a valid entry, false if not.
    /**
     * This is how you tell if your loop over entries is at the end.
     */
    bool
    isValid();

    /// Reset the iterator to the start of the map.
    void
    start();

    /// Return the name of the current map entry.
    QString
    currentKey();

    /// Return a Config reference to the current map entry.
    Config
    currentChild();

    // *INDENT-OFF*
  private:
    // *INDENT-ON*
    /// Private constructor enforces that MapIterators are only made by the Config class.
    MapIterator();

    Config::NodePtr node_;
    QMap<QString, Config::NodePtr>::const_iterator iterator_;
    bool iterator_valid_;
    friend class Config;
  };

  /// Return a new iterator for looping over key/value pairs.
  /**
   * The returned MapIterator is initialized to point at the start of the map.
   *
   * If this Config is Invalid or if its Node is not a Map, this returns a
   * MapIterator for which isValid() always returns false.
   */
  MapIterator
  mapIterator() const;

private:
  explicit Config(NodePtr node);

  static
  Config
  invalidConfig();

  /// If the node pointer is nullptr, this sets it to a new empty node.
  void
  makeValid();

  NodePtr node_;

  friend class MapIterator;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__CONFIG_HPP_
