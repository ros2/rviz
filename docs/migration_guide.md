# Migration Guide

This is a migration guide to facilitate migrating RViz plugins from RViz for ROS to RViz for ROS 2.
Please refer also to the ROS migration guide at https://github.com/ros2/ros2/wiki/Migration-Guide

## Initial migration

### Fixing includes (RViz has been split into multiple packages)

- Most user-facing functionality such as base classes for displays or tools are now located in `rviz_common`.
- The previous `ogre_helpers` as well as classes related to rendering are located in `rviz_rendering` or its subfolder `objects`.
- The default plugins and the robot are located in `rviz_default_plugins`.
- The Selection mechanism was moved to a subfolder `interaction` in `rviz_common`.
  The object picking behaviour was moved to a `ViewPicker`, which is also exposed via the display context.
- If the display extends the `MessageFilterDisplay`, switch to extending `RosTopicDisplay`.
  Message filters have not yet been ported.

### Fixing pluginlib includes

- The general mechanism of the pluginlib via `plugin_description.xml` has not changed.
- The base classes have been moved to `rviz_common`, e.g. `rviz_common/Display`.
  Names must refer to the correct class name and namespace, i.e. when copying a display's entry make sure to adapt names.

### Porting plugins which use Ogre directly

- The resources provided and loaded by RViz (such as certain materials or meshes) are now located in either the resource group with name `rviz_rendering` or the `Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME`.
- When providing additional resources for Ogre such as shaders, meshes or the like use the CMake macro `register_rviz_media_exports`.
  An example is provided in the CMakeLists and code in `rviz_rendering_tests`.

Note: It might be necessary to switch the Ogre backbone version in the future to a newer major version.
This might impact code already ported.

## ROS migration guidelines for porting from ROS to ROS 2

- Display plugins deriving from `rviz_common::RosTopicDisplay` automatically subscribe and unsubscribe to ROS topics.
  Further interaction with node handlers (ROS 2 executors) such as spinning the node are not necessary.
- Replace `ROS_ASSERT` by simple C++ `assert`.
- Error logging has not been ported to ROS 2.
  Use the macros `RVIZ_COMMON_LOG_ERROR`, `RVIZ_COMMON_LOG_INFO`, etc.
- Replace `ros::time` with the API provided by `rclcpp::Clock` and `rclcpp::Time`.
- RViz now uses `tf2` exclusively.

## Additional migration guidelines for plugins within RViz

### Note: This section may also be interesting for general plugin authors if they want to adhere to RViz style.

- Rename all headers from `.h` to `.hpp` as per the general guidelines.
- Include files into CMakeLists.
- Replace boost by the C++14 std-features (see https://github.com/ros2/ros2/wiki/Migration-Guide for details).
- For resources, either use the `resource_retriever` or, if they are located within the `AMENT_PREFIX_PATH`, `ament_index_cpp` can help.
  If this is not enough and a file system is needed, use Qt's version to avoid adding the boost dependency.
- Potentially adapt the default configuration of RViz `default.rviz`.
- Fix all linter errors. All subpackages of RViz currently run `cppcheck`, `cpplint`, `lint-cmake` and `uncrustify`.
- Fix all Visual Studio warnings.
  If the warnings occur within Ogre or other non-ROS third party libraries, pragmas may be used to ignore them.
  Otherwise either fix the warnings or open issues to the corresponding repositories.

### Migrate code to C++14

- Replace C style casts by C++ style casts.
- Replace raw pointers by smart pointers wherever possible.
  Exceptions are necessary when other libraries assume ownership of raw pointers.
  This mostly is the case for Ogre and Qt. Storing such objects inside smart pointers will lead to a double delete (once by the smart pointer, once by the owning library).
    - `Ogre::SceneNode` or `Ogre::SceneManager` are handled by Ogre directly.
    - `rviz_common::properties` are owned by Qt as long as a parent property is handed to the property upon creation.
- Replace index-for-loops by range-for-loops or iterator-for-loops if possible.
- Replace pointer assignments of 0, NULL by nullptr.
- Delete superfluous includes.

### Prepare plugins to run on Windows (Visibility Control)

Since RViz can now be used on Windows and Mac, it is necessary that code runs on those platforms as well.
The biggest issue for Windows is probably visibility control:
classes or methods from `rviz_rendering` and/or `rviz_common` to be used in other plugins must use the `RVIZ_RENDERING_PUBLIC` or the `RVIZ_COMMON_PUBLIC` modifier.
Failures will result in missing symbols while linking.

### Writing tests for displays

- To test displays, a special constructor must be provided to circumvent using `Display::initialize`, which needs a running ROS2 (`rclcpp::init()`).
- Expose `processMessage(...)` in the public interface of the class and call this during testing.
  This avoids using the API in RosTopicDisplay which is called by the subscribers and needs ROS.
- The Ogre dependency cannot be mocked.
  To work with it, a special testing setup is provided in `rviz_rendering`.
  In addition, helper methods to traverse the scene graph are provided in `rviz_rendering`.
- The tests need to be grouped with the other display tests, since the Ogre setup requires an actual display to be run.
  For now, display tests are only run on OSX at the OSRF Jenkins, but they will automatically be run on Linux if a physical display is present.
  To execute display tests on Windows, run `ament test --cmake-args -DEnableDisplayTests=True`.

When writing tests inside `rviz_default_plugins`, test setups can be reused: 
- For Displays, extend the `display_test_fixture`, which provides a fully set up mock of the Display Context and some convenience features to simulate correct transforms.
- For Tools, extend the `tool_test_fixture`, which provides a fully set up mock of Display Context as well as convenience methods for mouse interaction.
- For View Controllers, extend the `view_controller_test_fixture`, which provides a fully set up mock of Display Context as well as convenience methods to simulate mouse interaction

## End-to-end testing for RViz plugins

RViz now contains a small framework for end-to-end testing of displays, the package `rviz_visual_testing_framework`.
To use it, please refer to its documentation or the set of tests in `rviz_default_plugins`.

# List of important API changes

While porting RViz, the API is usually kept as is, but improved when necessary.
The following is an incomplete list of changes (to be expanded when porting more functionality):

## rviz_rendering

- `BillboardLine: newLine()` → `finishLine()` (identical functionality, but `finishLine()` can be called exactly `n` times, where `n` is the number of lines in the billboard line, whereas `newLine()` could only be called `n-1` times)
- `PointCloud: addPoints(...)` now takes vector iterators as arguments instead of pointer and size.
- `CovarianceVisual`: Does not inherit from `rviz_rendering::Object` anymore and has a much restricted interface.
Previously, this was only used by `rviz_common::properties::CovarianceProperty`.
To streamline interfaces, handling of visuals now needs to be done in the display: Set up a slot fetching data from the covariance property handing it over to CovarianceVisual (see OdometryDisplay for an example).

## rviz_common

- `SelectionManager::get3DPoint(...)` was moved to `ViewPicker::get3DPoint(...)`, which can be obtained from the `DisplayContext`.
- `SelectionManager::addHandler(...)` and similar functionality to work with selection handles has been moved to `HandlerManager`.
  When extending the `SelectionHandler`, registering and obtaining handles is still done automatically, so no changes should be necessary.
- The SelectionHandler needs to register its handle manually now. This can be done by calling the factory function defined in `selection_handler.hpp` instead of the constructor.
- When writing a new handler deriving from `SelectionHandler`, after creation, the handler needs to call `SelectionHandler::registerHandle()`.
In order to do this automatically, make the Factory method defined in `createSelectionHandler(...)` in `selection_handler.hpp` a friend of the new SelectionHandler.
For an example of how to do this, see PointCloudSelectionHandler.
Then the function can be used for handler creation and the SelectionHandler works as previously.
- In general, several functions within `SelectionManager` and `ViewPicker` have been moved to private API.
  If they are needed, please provide a pull request to the RViz repository explaining why this functionality is needed.
- `CovarianceProperty`: Previously used CovarianceVisual, now contains only a number of properties.
See `rviz_rendering::CovarianceVisual` for further information and OdometryDisplay in `rviz_default_plugins` for an example usage.
- The Display Context provides a whole host of new methods needed to write custom panels. The API has changed overall, but the functionality should only be extended.
