# rviz

This branch is still under construction and is targeted for ROS 2 and/or ROS 1 M-Turtle.

# Setup

Install ros2 (needed for ament at least and ros2's C++ api for the version of rviz in ros 2):

https://github.com/ros2/ros2/wiki/Installation

Currently the latest release (beta3) is not sufficient to build rviz so you need to build the ros2 master from source.

## Ubuntu

There will be a `setup.bash` file from your ros2 build you can source.
The current state of this branch requires a from-source build though.

Install the following Ubuntu packages: 

```
apt install libxaw7-dev libgles2-mesa-dev libglu1-mesa-dev qtbase5-dev libyaml-cpp-dev
```

Build `assimp` 3.3.1 from [source](https://github.com/assimp/assimp/archive/v3.3.1.tar.gz) (standard cmake build) as the available Ubuntu package does currently not work.

Source the setup file before continuing:

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
