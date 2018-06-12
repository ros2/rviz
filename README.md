# rviz

This branch is currently contained in the main `ros2.repos` file of ROS 2 and can be used for ROS 2.
The latest release will be available with your ROS 2 download.

# Features in the current master version

## Features present in rviz for ROS and missing in rviz for ROS 2

Tools:
- Interactive Markers

Displays:
- AxesDisplay
- DepthCloudDisplay
- EffortDisplay
- FluidPressureDisplay
- IlluminanceDisplay
- InteractiveMarkerDisplay
- PoseWithCovarianceStampedDisplay
- RangeDisplay
- RelativeHumidityDisplay
- TemperatureDisplay
- WrenchDisplay

Other features:
- Filtering of Topic lists by topic type
- Message filters
- Image transport features

If you would like to see those features in rviz, feel free to add a pull request.
Make sure to read the developer guide below and the migration guide.

## Features new in rviz for ROS 2

None, yet.

# Developer Guide

## Setup: Building from source

The simplest way to build from source is to use the official installation guide, since rviz is part of the official ROS 2 repos file.

https://github.com/ros2/ros2/wiki/Installation

### Building RViz in a separate folder

When developing for rviz, it can be beneficial to build it in a separate folder. 

**Note:** When building the current ros2 branch from source, the latest ROS 2 release for all dependencies might not be sufficient. Make sure to have a source build of ROS 2 available (see installation procedure above).

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
$ ament build
```

Or specific packages with command, for example just `rviz_ogre_vendor` and then `rviz_rendering`:

```
$ ament build --only rviz_ogre_vendor
$ ament build --only rviz_rendering
```

More instructions and examples to come.

In addition to the [ROS 2 Developer Guide](https://github.com/ros2/ros2/wiki/Developer-Guide) we suggest the following.

## Testing

Main rationale here is to create code that can be well tested by avoiding highly coupled components.

* Avoid free functions (cannot be mocked).
* Create abstract base classes (interface) for dependencies.
  (This allows for a mock to be completely independent of the actual implementing class.)
* Use only the interface in the dependent code.
* Specify dependencies as a constructor argument.
* Prefer `std::unique_ptr` for storing the dependency instead of a raw pointer.

## Migration Guide

When migrating from rviz to rviz2, please see the more extensive [migration guide](https://github.com/ros2/rviz/migration_guide.md).

