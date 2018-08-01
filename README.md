# RViz

This branch is currently contained in the main `ros2.repos` file of ROS 2 and can be used for ROS 2.
The latest release will be available with your ROS 2 download.

ROS 2 does not have a wiki yet. To learn about RViz and its functionality, please refer to the ROS RViz [wiki page](http://www.ros.org/wiki/rviz).

## Features

### Already ported
These features have already been ported from `ros-visualization/rviz` to `ros2/rviz`.
The basic documentation can still be found on the RViz [wiki page](http://www.ros.org/wiki/rviz).
For some displays, the [documentation is updated](docs/FEATURES.md).

| Displays              | Tools         | View Controller       | Panels          |
| --------------------- | ------------- | --------------------- | --------------- |
| Camera                | Move Camera   | Orbit                 | Displays        |
| Grid                  | Focus Camera  | XY Orbit              | Help            |
| Grid Cells            | Measure       | First Person          | Selections      |
| Image                 | Select        | Third Person Follower | Tool Properties |
| Laser Scan            | 2D Nav Goal   | Top Down Orthographic | Views           |
| Map                   | Publish Point |                       |                 |
| Marker                | Initial Pose  |
| Marker Array          |
| Odometry              |
| Point Cloud (1 and 2) |
| Point                 |
| Polygon               |
| Pose                  |
| Pose Array            |
| Range                 |
| Robot Model           |
| TF                    |

### Not yet ported
These features have not been ported to `ros2/rviz` yet.

| Displays             | Tools        | Panels |
| -------------------- | ------------ | ------ |
| Axes                 | Interact     | Time   |
| DepthCloud           |
| Effort               |
| Fluid Pressure       |
| Illuminance          |
| Interactive Marker   |
| Oculus               |
| Pose With Covariance |
| Relative Humidity    |
| Temperature          |
| Wrench               |

Other features:
- Filtering of Topic lists by topic type
- Message filters
- Image transport features

In case you wished to see those features in RViz for ROS 2, feel free to add a pull request.
Make sure to read the developer guide below and the migration guide.

### New features
#### Pluggable transformation library

In RViz for ROS 1 the frames transformation library used is tf2 (detailed information about it can be found [here](http://wiki.ros.org/tf2)).
In RViz for ROS 2 the frames transformation library is now pluggable, meaning that different transformation library plugins can be loaded and changed dynamically in the gui.
Developers can create and use their own plugins to provide custom transformation behavior.

Two plugins are bundled with RViz:
- a plugin for tf2 (`TFFrameTransformer`, in `rviz_default_plugins`), which provides the standard tf2 functionality and which is used as a default
- a trivial plugin (`IdentityFrameTransformer`, in `rviz_common`), which always performs identity transforms.
  This plugin is used by default if the tf2 plugin is not available and no other valid plugin is specified.

As anticipated, in order for the user to choose the plugin to use, RViz provides a dedicated panel: the `Transformation` panel.

Note: Not all transformation plugins are necessarily compatible with all RViz displays (e.g. some of the default displays, like the TF display, can only work with tf2).
In order to take this possibility into account, the `TransformerGuard` class is provided.
Adding it to a display ensures that the display will be disabled and won't function in case the wrong transformer is used.

More detailed information on how to write a transformation plugin and on how to handle transformation specific displays can be found in the [plugin development guide](docs/plugin_development.md).

## Developer Guide

### Build
#### Building RViz together with ROS 2

The simplest way to build from source is to use the official installation guide, since RViz is part of the official ROS 2 repos file.

https://github.com/ros2/ros2/wiki/Installation

#### Building RViz in a separate workspace

When developing for RViz, it can be beneficial to build it in a separate workspace.

**Note:** When building the current ros2 branch from source, the latest ROS 2 release for all dependencies might not be sufficient: it could be necessary to build the ROS 2 master branch.
Make sure to have a source build of ROS 2 available (see installation procedure above).

Create a new workspace:

```
$ mkdir -p ~/rviz2_ws/src
$ cd ~/rviz2_ws/src
```

Clone these repositories into the source folder:

```
$ git clone https://github.com/ros2/rviz.git
```

Then build all the packages with this command:

```
$ colcon build --merge-install
```
The `--merge-install` flag is optional but ensures a cleaner environment which is helpful for development.

More instructions and examples to come.

In addition to the [ROS 2 Developer Guide](https://github.com/ros2/ros2/wiki/Developer-Guide) we suggest the following.

### Testing

Main rationale here is to create code that can be well tested by avoiding highly coupled components.

* Avoid free functions (cannot be mocked).
* Create abstract base classes (interface) for dependencies.
  (This allows for a mock to be completely independent of the actual implementing class.)
* Use only the interface in the dependent code.
* Specify dependencies as a constructor argument.
* Prefer `std::unique_ptr` for storing the dependency instead of a raw pointer.

### Migration

When migrating from `ros-visualization/rviz` to `ros2/rviz`, please see the more extensive [migration guide](docs/migration_guide.md).

### Plugin Development

Plugins can extend RViz at different extension points:
- Displays
- Panels
- Tools
- Frames transformation library
- View Controllers

More information on writing plugins can be found in the [plugin development guide](docs/plugin_development.md).
