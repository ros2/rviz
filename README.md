# rviz

This branch is still under construction and is targeted for ROS 2 and/or ROS 1 M-Turtle.

# Setup

Install ros2 (needed for ament at least and ros2's C++ api for the version of rviz in ros 2):

https://github.com/ros2/ros2/wiki/Installation

Currently the latest release (beta3) is not sufficient to build rviz so you need to build the ros2 master from source.

## Prerequisites

### Ubuntu

There will be a `setup.bash` file from your ros2 build you can source.
The current state of this branch requires a from-source build though.

Install the following Ubuntu packages:

```
apt install libxaw7-dev libgles2-mesa-dev libglu1-mesa-dev qt5-default libyaml-cpp-dev libcurl4-openssl-dev
```

Source the setup file before continuing:

```
$ source path/to/ros2/install/setup.bash
```

### Windows

**Note** Make sure to install all dependencies in either 32 bit or 64 bit version and do not mix.

This setup was tested for Windows 10 x64.

#### Build CURL 7.56.0
* Download CURL sources from [GitHub](https://github.com/curl/curl/releases/tag/curl-7_56_0)
* Extract to local folder (e.g. to `C:\ros2\curl-7.56.0`)
* Create and change to build folder (e.g. `C:\ros2\curl-7.56.0\build`)
    * Configure CMake: `cmake -G "Visual Studio 15 2017 Win64" ../`
    * Build the project: `cmake --build . --config Debug`
    * Install to `C:\Program Files`: `cmake --build . --config Debug --target Install`

#### Get Boost (No build required)
* Download Boost sources from https://dl.bintray.com/boostorg/release/1.65.1/source/
* Extract to local folder (e.g. to `C:\ros2\boost_1_65_1`)

#### Build yaml-cpp
* Download yaml-cpp sources from [GitHub](https://github.com/jbeder/yaml-cpp/releases/tag/yaml-cpp-0.5.3)
* Extract to local folder (e.g. to `C:\ros2\yaml-cpp-release-0.5.3`)
* Create and change to build folder (e.g. `C:\ros2\yaml-cpp-release-0.5.3\build`)
    * Configure CMake: `cmake -G "cmake -G "Visual Studio 15 2017 Win64" ../ -DBoost_INCLUDE_DIR=C:\ros2\boost_1_65_1`
    * Build the project: `cmake --build . --config Debug`
    * Install to `C:\Program Files`: `cmake --build . --config Debug --target Install`

#### Setup environment
* add Qt binary files to PATH (e.g. `C:\Qt\5.9.1\msvc2017_64\bin`)
* set QT_QPA_PLATFORM_PLUGIN_PATH environment variable (e.g. `C:\Qt\5.9.1\msvc2017_64\plugins\platforms`)
* Add Curl and yaml-cppto the CMAKE_PREFIX_PATH environment variable
    * Example: `C:\Program Files\CURL;C:\Program Files\YAML_CPP`
* Add Curl binary to PATH (e.g. `C:\Program Files\CURL\bin`)
* Set BOOST_INCLUDEDIR environment variable to Boost include directory (e.g. `C:\ros2\boost_1_65_1`)
* Add patch.exe to PATH (e.g. from Git Bash, `C:\Program Files\Git\usr\bin`)
* (For Testing) Add Cppcheck binary to PATH (e.g. `C:\Program Files\Cppcheck`)

Source the setup file before continuing:

```
$ call path/to/ros2/install/setup.bat
```

## Building RViz

Create a new workspace:

```
$ mkdir -p ~/rviz2_ws/src
$ cd ~/rviz2_ws/src
```

Clone these repositories into the source folder:

```
$ git clone https://github.com/ros2/rviz.git
$ git clone https://github.com/ros/pluginlib.git -b ros2
$ git clone https://github.com/ros2/tinyxml2_vendor.git
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
