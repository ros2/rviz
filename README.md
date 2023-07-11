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
| Axes                  | Move Camera   | Orbit                 | Displays        |
| Camera                | Focus Camera  | XY Orbit              | Help            |
| DepthCloud            | Measure       | First Person          | Selections      |
| Effort                | Select        | Third Person Follower | Time            |
| Fluid                 | 2D Nav Goal   | Top Down Orthographic | Tool Properties |
| Grid                  | Publish Point |                       | Views           |
| Grid Cells            | Initial Pose  |
| Illuminance           | Interact      |
| Image                 |
| Interactive Marker    |
| Laser Scan            |
| Map                   |
| Marker                |
| Marker Array          |
| Odometry              |
| Point Cloud (1 and 2) |
| Point                 |
| Polygon               |
| Pose                  |
| Pose Array            |
| Pose With Covariance  |
| Range                 |
| Relative Humidity     |
| Robot Model           |
| Temperature           |
| TF                    |
| Wrench                |


### Not yet ported
Other features:
- Stereo

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

https://docs.ros.org/en/rolling/Installation.html

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

In addition to the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html) we suggest the following.

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

## Icon Copyrights

This package contains Public Domain icons downloaded from http://tango.freedesktop.org/releases/.

Upstream Authors (2005-2009):

- Ulisse Perusin `<uli.peru@gmail.com>`
- Steven Garrity `<sgarrity@silverorange.com>`
- Lapo Calamandrei `<calamandrei@gmail.com>`
- Ryan Collier `<rcollier@novell.com>`
- Rodney Dawes `<dobey@novell.com>`
- Andreas Nilsson `<nisses.mail@home.se>`
- Tuomas Kuosmanen `<tigert@tigert.com>`
- Garrett LeSage `<garrett@novell.com>`
- Jakub Steiner `<jimmac@novell.com>`

Other icons and graphics contained in this package are released into the
Public Domain as well.

Authors (2012-2017):

- David Gossow
- Chad Rockey
- Kei Okada
- Julius Kammerl
- Acorn Pooley
- Rein Appeldoorn

Copyright notice for all icons and graphics in this package:

```
Public Domain Dedication

Copyright-Only Dedication (based on United States law) or Public Domain
Certification

The person or persons who have associated work with this document (the
"Dedicator" or "Certifier") hereby either (a) certifies that, to the best
of his knowledge, the work of authorship identified is in the public
domain of the country from which the work is published, or (b)
hereby dedicates whatever copyright the dedicators holds in the work
of authorship identified below (the "Work") to the public domain. A
certifier, moreover, dedicates any copyright interest he may have in
the associated work, and for these purposes, is described as a
"dedicator" below.

A certifier has taken reasonable steps to verify the copyright
status of this work. Certifier recognizes that his good faith efforts
may not shield him from liability if in fact the work certified is not
in the public domain.

Dedicator makes this dedication for the benefit of the public at
large and to the detriment of the Dedicator's heirs and successors.
Dedicator intends this dedication to be an overt act of relinquishment
in perpetuity of all present and future rights under copyright law,
whether vested or contingent, in the Work. Dedicator understands that
such relinquishment of all rights includes the relinquishment of all
rights to enforce (by lawsuit or otherwise) those copyrights in the
Work.

Dedicator recognizes that, once placed in the public domain, the Work
may be freely reproduced, distributed, transmitted, used, modified,
built upon, or otherwise exploited by anyone for any purpose, commercial
or non-commercial, and in any way, including by methods that have not
yet been invented or conceived.
```

Source: http://creativecommons.org/licenses/publicdomain/
