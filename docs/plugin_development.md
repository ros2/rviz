# User's Guide to plugin development

This is intended as a guide for people wishing to develop custom plugins.


## Plugin Development

### Extension Points

Plugins can extend RViz at different extension points:

| plugin type                   | base type                                       |
| ----------------------------- | ----------------------------------------------- |
| Display                       | `rviz_common::Display`                          |
| Panel                         | `rviz_common::Panel`                            |
| Tool                          | `rviz_common::Tool`                             |
| Frames transformation library | `rviz_common::transformation::FrameTransformer` |
| View Controller               | `rviz_common::ViewController`                   |

Every plugin must be of the corresponding base type in order to be recognized by RViz.
Refer to the documentation of the relevant base class for a detailed API description.

### Plugin Basics

* In order to write your own plugin, set up a regular ROS 2 workspace (see the [official documentation]("https://github.com/ros2/ros2/wiki/Colcon-Tutorial") for further help).
* You will need to link your program against `rviz_common` and probably also against `rviz_rendering` and `rviz_default_plugins`.
* To let the plugin loader find your plugin, you will need to invoke the `PLUGINLIB_EXPORT_CLASS` macro.
  For instance, if your plugin is called "myplugin::Plugin" with base class `rviz_common::Display`, you will need to include the following into your `.cpp` file:
```
    #include <pluginlib/class_list_macros.hpp>
    PLUGINLIB_EXPORT_CLASS(myplugin::Plugin, rviz_common::Display)
```
* You need to link and compile against the `pluginlib` package. Further CMake options that might be relevant (see `rviz_default_plugins` CMakeLists):
```
# Causes the visibility macros to use dllexport rather than dllimport (for Windows, when your plugin should be used as library)
target_compile_definitions(rviz_default_plugins PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
```
* You need to write a `plugin_description.xml` file which contains the information necessary for the pluginlib to load your plugin at runtime.
  See `rviz_default_plugins/plugins_description.xml` for an example (the syntax is the same as for the old RViz)
* Export the plugin description file via
```
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)
```
This should make sure that your plugins are found by rviz_common at runtime as described [here](https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html).

### Writing a display plugin

* In order to write a display plugin, derive from either `rviz_common::Display` or `rviz_common::RosTopicDisplay<MessageType>`.

#### Writing a display without ros subscription

When writing a display without subscription to ros, derive from `rviz_common::Display`.

* The Display class provides an `Ogre::SceneNode`, which can be used to add visual objects.
* It provides convenience methods to set and delete Status messages
* When you need any Ogre or SceneNode setup, override the `onInitilize()` method.

#### Writing a display with subscription

When writing a display for a topic type, derive from `rviz_common::RosTopicDisplay<MessageType>`.
The RosTopicDisplay is templated on the message type of this display:

* RosTopicDisplay derives from Display and thereby contains all of the functionality described in the previous section
* It handles subscribing and unsubscribing to topics of its template type
* It provides two properties: a property to choose the topic and a property to choose the QoS profile ("Quality of Service", currently "reliable" or "unreliable")
* It provides convenient methods to receive messages and do initial checking. Override the `processMessage` to add behaviour on receiving a new message

#### Providing Ogre media files

It is possible to add your own meshes, scripts, etc. to RViz.
To make them available at runtime, use the cmake macro `register_rviz_ogre_media_exports` defined in `rviz_rendering`
For instance, if you want to register a folder `test_folder/scripts` which resides in your project's source folder, write
```
register_rviz_ogre_media_exports(DIRECTORIES "test_folder/scripts")
```
*Note:* If you want to export folder hierarchies, each folder needs to be exported separately. Folders within exported folders are not automatically available.

#### Writing a display which can only work with one transformation plugin

Some of the default displays (e.g. TF display or LaserScan display) can only work with tf2 as transformation framework.
Similarly, as a developer, you may have written your own transformation plugin which cannot work with every display, or your own display which cannot work with every transformation plugin.
The class `rviz_default_plugins::transformation::TransformerGuard` helps you to handle these possibilities.

As explained below, every transformation plugin implements the base class `rviz_common::transformation::FrameTransformer`.
The `TransformerGuard` class is templated on the type of this implementation and will make sure that the display by which it is owned will be disabled if the transformer currently in use is of a different type.

When writing a display that is supposed to be working only with a specific transformation plugin, you can add a templated instance of `TransformerGuard` as a display member.
The constructor of this class takes two argument: a raw pointer to the display that owns it, and the name of the transformer it is supposed to work with (needed to correctly set the error status).
Ideally the `TransformerGuard` object is initialized in the constructor of the display.

In order to make the `TransformerGuard` object work correctly, you need to call the method `TransformerGuard::initialize(rviz_common::DisplayContext * context)` inside the display `onInitialize()` method, passing as parameter the `context_` member of the display.
Doing this will make sure that every time the current transformer plugin is of the wrong type, the display will be disabled and an appropriate error message will be shown in the display status.

In addition to this, the `TransformerGuard` class also provides the `checkTransformer()` method, which returns `true` if the currently usd transformer is of the allowed type, and `false` otherwise.
This method can be used by the display every time you want it to behave differently according to what kind of transformation plugin is used at the moment.

For a concrete example of how all this is done, you can have a look, for example, at the TF or the LaserScan display, in `rviz_default_plugins` which, as said, only work with tf2.

### Writing a panel plugin

To write a custom panel, derive from `rviz_common::Panel`.

### Writing a tool plugin

* When writing a tool, derive from `rviz_common::Tool`
* Hotkeys for activating and deactivating tools can be set by setting the member variable `shortcut_key_` to the char of your choice
* Adding properties to the "Tool Properties" panel is possible by passing the return value of the function `getPropertyContainer()` as parent to the property in question (see `rviz_default_plugins::tools::MeasureTool` for an example)
* Custom cursors, icons, etc. for use in your plugins are available through `rviz_common/load_resource`

### Writing a view controller plugin

* In order to write a custom view controller which should be independent of tf frames, derive from `rviz_common::ViewController`
* If your view controller should be able to track tf frames in a scene, derive from `rviz_common::FramePositionTrackingViewController`, which already contains convenience functionality for tracking target frames
* If your custom view controller orbits a focal point, it might also be beneficial to derive from `rviz_default_plugins::OrbitViewController`

### Writing a transformation library plugin

- To write a transformer plugin, you must implement the `rviz_common::transformation::FrameTransformer` class (refer to the API documentation contained in the header file).
- If for your plugin you also need extra functionality, or you want to offer direct access to some parts of the library itself, you can also implement a subclass of `rviz_common::transformation::TransformerLibraryConnector` (see, for example, `rviz_default_plugins::transformation::TFWrapper`).

## Overview of RViz API

### rviz_rendering

The `rviz_rendering` package is supposed to contain all functionality referring to rendering:

* Visuals and objects to be added to the scene graph such as arrows, shapes or text objects in the subfolder `objects` (many of those objects were ported from the folder `ogre_helpers`)
* The render window, including functions exposing some of its internals (RenderWindowOgreAdapter).
  If possible, refrain from using the RenderWindowOgreAdapter, as it might be deprecated and deleted in the future
* Convenience functions to work with materials (`material_manager.hpp`) or other ogre related functionality (e.g. ray tracing in `viewport_projection_finder.hpp`)
* Convenience classes for testing, allowing to set up a working ogre environment and helpers for scene graph introspection

### rviz_common

The `rviz_common` package contains the bulk of RViz useful for doing plugin development:

* Main application and rendering queues (not exposed)
* Main entry points for plugin development - base classes for panels, view controllers, displays and tools
* Convenience classes to work with properties in the various views (such as the display panel) located in `rviz_common/properties`
* Main classes for selectable types, the SelectionHandlers (located in `rviz_common/interaction`)
* Access points to ROS 2.
  Currently, RViz uses only one node, which can be accessed via `ros_integration`.
  In the future, further changes may be necessary to fully abstract ROS 2 access into `ros_integration`

### rviz_default_plugins

The `rviz_default_plugins` contains all plugins (view controllers, tools, displays and in the future, panels) shipped with RViz (most of them ported from the `default_plugins` folder of RViz).

* When developing simple plugins it is not necessary to use anything in this package.
* When developing more complex plugins similar to existing plugins it might be beneficial to use or even derive from classes contained in this package to simplify your development.

### rviz_visual_testing_framework

The `rviz_visual_testing_framework` contains the backbone to writing visual tests for plugins.
It will only ever be necessary to use this package as a test dependency if you want to write automated screenshot tests.
Please see the documentation in the package for further help.

### rviz2

This package contains the main program startup and entry point for nodes (not exposed).
