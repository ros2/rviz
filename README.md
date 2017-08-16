# rviz

This branch is still underconstruction and is targetd for ROS 2 and/or ROS 1 M-Turtle.

# Setup

Install ros2 (needed for ament at least and ros2's C++ api for the version of rviz in ros 2):

https://github.com/ros2/ros2/wiki/Installation

Whether you install from source or from a binary, there will be a `setup.bash` file you can source.
Source that file before continuing:

```
$ source path/to/ros2/install/setup.bash
```

Then create a new workspace:

```
$ mkdir -p ~/rviz2_ws/src
$ cd ~/rviz2_ws/src
```

Clone this repository into the source folder:

```
$ git clone https://github.com/ros-visualization/rviz.git -b ros2
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
