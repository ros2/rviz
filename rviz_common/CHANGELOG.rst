^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

7.0.3 (2019-11-13)
------------------

7.0.2 (2019-10-23)
------------------

7.0.1 (2019-10-04)
------------------

7.0.0 (2019-09-27)
------------------
* Introduce QoS property (`#409 <https://github.com/ros2/rviz/issues/409>`_)
  A container of properties related to QoS settings. Replaces queue size and unreliable properties.
* Migrate InteractiveMarkerDisplay (`#457 <https://github.com/ros2/rviz/issues/457>`_)
* Rename 2d Nav Goal to 2d Goal Pose (`#455 <https://github.com/ros2/rviz/issues/455>`_)
    * Rename nav pose tool to goal pose tool
    * Rename topic for goal pose tool from "move_base_simple/goal" to "goal_pose"
* Do not select interactive markers when mousing over them (`#451 <https://github.com/ros2/rviz/issues/451>`_)
* Migrate Interaction Tool (`#423 <https://github.com/ros2/rviz/issues/423>`_)
* Upgrade from Ogre 1.10 to Ogre 1.12.1 (`#394 <https://github.com/ros2/rviz/issues/394>`_)
* Re-enable use of tf message filter (`#375 <https://github.com/ros2/rviz/issues/375>`_)
* Static analysis cleanup for rviz_common (`#431 <https://github.com/ros2/rviz/issues/431>`_)
* Fix deprecation warnings with new Qt (`#435 <https://github.com/ros2/rviz/issues/435>`_)
* FrameTransformer implements tf2::BufferCoreInterface and tf2_ros::AsyncBufferInterface (`#422 <https://github.com/ros2/rviz/issues/422>`_)
* Use geometry_msgs types instead of custom types (`#426 <https://github.com/ros2/rviz/issues/426>`_)
* Remove -Werror from defualt compiler options (`#420 <https://github.com/ros2/rviz/issues/420>`_)
* Use node to create clock used to stamp publications (`#407 <https://github.com/ros2/rviz/issues/407>`_)
* Remove blank lines that latest uncrustify doesn't like (`#411 <https://github.com/ros2/rviz/issues/411>`_)
* Contributors: Emerson Knapp, Hunter L. Allen, Jacob Perron, Martin Idel, Scott K Logan, Shivesh Khaitan, Steven Macenski, William Woodall

6.1.1 (2019-05-29)
------------------

6.1.0 (2019-05-20)
------------------
* Updated to use the new specification for types from the ROS node graph API. (`#387 <https://github.com/ros2/rviz/issues/387>`_)
* Contributors: Jacob Perron

6.0.0 (2019-05-08)
------------------
* Made changes to avoid newly deprecated API's related to publishers and subscriptions. (`#399 <https://github.com/ros2/rviz/issues/399>`_)
* Updated to be compatible with new QoS settings.  (`#392 <https://github.com/ros2/rviz/issues/392>`_)
* Fixed a crash on shutdown by properly freeing the ``transformation_manager``. (`#386 <https://github.com/ros2/rviz/issues/386>`_)
* Contributors: M. M, Michael Jeronimo, William Woodall

5.1.0 (2019-01-14)
------------------
* Fix errors from uncrustify v0.68 (`#366 <https://github.com/ros2/rviz/issues/366>`_)
* Contributors: Jacob Perron, William Woodall

5.0.0 (2018-12-04)
------------------
* Moved rviz_yaml_cpp_vendor into it's own repo (`#361 <https://github.com/ros2/rviz/issues/361>`_)
* Exported libraries to trigger hooks. (`#358 <https://github.com/ros2/rviz/issues/358>`_)
* Added virtual destructors for WindowManagerInterface and ViewportProjectionFinder (`#356 <https://github.com/ros2/rviz/issues/356>`_)
* Changed to use spin_some with max_duration (`#354 <https://github.com/ros2/rviz/issues/354>`_)
  * spin_once() limits the number of callbacks that can be called based on how quickly the gui updates.
  * This results in lag when displaying tf frames.
  * Use spin_some(max_duration) to execute all of the work available.
* Made the transformation framework used by rviz pluggable. (`#346 <https://github.com/ros2/rviz/issues/346>`_)
* Fixed wrong name of InitialPose plugin in default config (`#352 <https://github.com/ros2/rviz/issues/352>`_)
* Added hotkeys (also to help) (`#312 <https://github.com/ros2/rviz/issues/312>`_)
* Migrated pose estimate tool (`#329 <https://github.com/ros2/rviz/issues/329>`_)
* Changed to now pass clock to tf2_ros::buffer (`#340 <https://github.com/ros2/rviz/issues/340>`_)
* Changted to always build all tests and skip execution if not supported (`#342 <https://github.com/ros2/rviz/issues/342>`_)
* Reverted workaround for identity transform after geometry2 fix (`#343 <https://github.com/ros2/rviz/issues/343>`_)
* Reverted "Add simple mechanism to automatically convert old configs" (`#337 <https://github.com/ros2/rviz/issues/337>`_)
  * No longer necessary for C-Turtle
* Minor cleanup and fixes (`#336 <https://github.com/ros2/rviz/issues/336>`_)
  * Fix environment for colcon builds (no appends necessary)
  * Fix warning in visual_test_fixture.cpp and add -Werror option in CMakeLists.txt
  * Fix Qt moc warning for virtual signal
* Fixed missing status in laser scan display (`#335 <https://github.com/ros2/rviz/issues/335>`_)
  * Show status error when transform failed
  * Modify logging behaviour
* Updates due to uncrustify 0.67 (`#333 <https://github.com/ros2/rviz/issues/333>`_)
  * fix closing block and tamplete indentation to comply with uncrustify 0.67
  * add space between reference and variable name
  * space before opening bracket
  * fix indend of inherited class
  * fix indent of code blocks
  * no space around -> operator
  * restore original spacing
* Contributors: Andreas Greimel, Kartik Mohta, Martin Idel, Michael Carroll, Mikael Arguedas, Shane Loretz, Steven! Ragnarök

4.0.1 (2018-06-28)
------------------

4.0.0 (2018-06-27)
------------------
* Polished selection behavior. (`#315 <https://github.com/ros2/rviz/issues/315>`_)
* Fixed invalid color profile in PNGs. (`#306 <https://github.com/ros2/rviz/issues/306>`_)
* Updated use of tf2 so that it reused the rviz node. (`#290 <https://github.com/ros2/rviz/issues/290>`_)
* Made the display status not editable. (`#316 <https://github.com/ros2/rviz/issues/316>`_)
* Fixed some bugs causing RViz to crash on macOS. (`#319 <https://github.com/ros2/rviz/issues/319>`_)
* Fixed a segfault that could occur on empty frames. (`#313 <https://github.com/ros2/rviz/issues/313>`_)
* Moved the selection icon to ``rviz_common``. (`#314 <https://github.com/ros2/rviz/issues/314>`_)
* Polished more tests and migrate or delete old tests. (`#289 <https://github.com/ros2/rviz/issues/289>`_)
* Migrated the map display. (`#267 <https://github.com/ros2/rviz/issues/267>`_)
* Migrated Marker Array Display. (`#296 <https://github.com/ros2/rviz/issues/296>`_)
* Migrated 2D Nav Goal tool. (`#294 <https://github.com/ros2/rviz/issues/294>`_)
* Fixed a memory leak in shutdown of ros client abstraction. (`#301 <https://github.com/ros2/rviz/issues/301>`_)
* Implemented a workaround for the missing identity transform when transforming from a frame to itself. (`#298 <https://github.com/ros2/rviz/issues/298>`_)
* Fixed the splash screen and help panel. (`#277 <https://github.com/ros2/rviz/issues/277>`_)
* Migrated the Odometry display. (`#275 <https://github.com/ros2/rviz/issues/275>`_)
* Homogenized behavior of rviz when a tf transform fails. (`#292 <https://github.com/ros2/rviz/issues/292>`_)
* Changed rviz so that only a single ros node is used. (`#197 <https://github.com/ros2/rviz/issues/197>`_)
* Migrated the XYOrbit View Controller. (`#282 <https://github.com/ros2/rviz/issues/282>`_)
* Fixed select and measure tool behavior on macOS Retina displays. (`#283 <https://github.com/ros2/rviz/issues/283>`_)
* Fixed a segfault in the selection manager tests. (`#284 <https://github.com/ros2/rviz/issues/284>`_)
* Fixed rviz application so it shows an icon on macOS's toolbar when running. (`#272 <https://github.com/ros2/rviz/issues/272>`_)
* Moved the ``rviz_common::Panel`` into a public header. (`#265 <https://github.com/ros2/rviz/issues/265>`_)
* Migrated the orthographic view controller. (`#270 <https://github.com/ros2/rviz/issues/270>`_)
* Restored most of the command line options for rviz. (`#255 <https://github.com/ros2/rviz/issues/255>`_)
* Migrated tool properties panel. (`#251 <https://github.com/ros2/rviz/issues/251>`_)
* Finished point cloud refactoring and testing. (`#250 <https://github.com/ros2/rviz/issues/250>`_)
* Migrated the select tool. (`#256 <https://github.com/ros2/rviz/issues/256>`_)
* Introduced visual testing framework for rviz. (`#209 <https://github.com/ros2/rviz/issues/209>`_)
* Fixed "display" tests and enable when the test environment allows. (`#186 <https://github.com/ros2/rviz/issues/186>`_)
* Restored use of icons throughout rviz. (`#235 <https://github.com/ros2/rviz/issues/235>`_)
* Migrated marker display. (`#229 <https://github.com/ros2/rviz/issues/229>`_)
* Changed the default position of the camera in RenderPanel. (`#205 <https://github.com/ros2/rviz/issues/205>`_)
* Migrated RobotModelDisplay. (`#210 <https://github.com/ros2/rviz/issues/210>`_)
* Fixed a possible null pointer is dereference. (`#178 <https://github.com/ros2/rviz/issues/178>`_)
  * Signed-off-by: Chris Ye <chris.ye@intel.com>
* Migrated camera display (`#183 <https://github.com/ros2/rviz/issues/183>`_)
* Updated Ogre to 1.10.11 (`#181 <https://github.com/ros2/rviz/issues/181>`_)
* Migrated TF Display. (`#182 <https://github.com/ros2/rviz/issues/182>`_)
* Migrated ImageDisplay. (`#164 <https://github.com/ros2/rviz/issues/164>`_)
* Introduced ROS interface abstraction to improve testability. (`#156 <https://github.com/ros2/rviz/issues/156>`_)
* Re-enabled and fixed rviz configuration file loading. (`#167 <https://github.com/ros2/rviz/issues/167>`_)
* Fixed a bug caused by a missing break in switch statement. (`#158 <https://github.com/ros2/rviz/issues/158>`_)
* Migrated code for point cloud displays to ``rviz_default_plugins``. (`#153 <https://github.com/ros2/rviz/issues/153>`_)
* Contributors: Alessandro Bottero, Andreas Greimel, Andreas Holzner, Chris Ye, Dirk Thomas, Martin Idel, Mikael Arguedas, Steven! Ragnarök, Tim Rakowski, William Woodall

3.0.0 (2018-02-07)
------------------
* Fixed compilation errors and runtime issues on Windows. (`#175 <https://github.com/ros2/rviz/issues/175>`_)
* Fixed an issue with docking windows initially.
* Fixed a potential memory leak.
* Fixed a bug which caused rviz to crash when removing a display. (`#191 <https://github.com/ros2/rviz/issues/191>`_)
  * The crash occurred when adding a camera display and then deleting any display that was created before adding the camera display.
* Contributors: Andreas Greimel, Andreas Holzner, Johannes Jeising, Martin Idel, Steven! Ragnarok, William Woodall

1.12.11 (2017-08-02)
--------------------

1.12.10 (2017-06-05 17:37)
--------------------------

1.12.9 (2017-06-05 14:23)
-------------------------

1.12.8 (2017-05-07)
-------------------

1.12.7 (2017-05-05)
-------------------

1.12.6 (2017-05-02)
-------------------

1.12.5 (2017-05-01)
-------------------

1.12.4 (2016-10-27)
-------------------

1.12.3 (2016-10-19)
-------------------

1.12.2 (2016-10-18)
-------------------

1.12.1 (2016-04-20)
-------------------

1.12.0 (2016-04-11)
-------------------

1.11.14 (2016-04-03)
--------------------

1.11.13 (2016-03-23)
--------------------

1.11.12 (2016-03-22 19:58)
--------------------------

1.11.11 (2016-03-22 18:16)
--------------------------

1.11.10 (2015-10-13)
--------------------

1.11.9 (2015-09-21)
-------------------

1.11.8 (2015-08-05)
-------------------

1.11.7 (2015-03-02)
-------------------

1.11.6 (2015-02-13)
-------------------

1.11.5 (2015-02-11)
-------------------

1.11.4 (2014-10-30)
-------------------

1.11.3 (2014-06-26)
-------------------

1.11.2 (2014-05-13)
-------------------

1.11.1 (2014-05-01)
-------------------

1.11.0 (2014-03-04 21:40)
-------------------------

1.10.14 (2014-03-04 21:35)
--------------------------

1.10.13 (2014-02-26)
--------------------

1.10.12 (2014-02-25)
--------------------

1.10.11 (2014-01-26)
--------------------

1.10.10 (2013-12-22)
--------------------

1.10.9 (2013-10-15)
-------------------

1.10.7 (2013-09-16)
-------------------

1.10.6 (2013-09-03)
-------------------

1.10.5 (2013-08-28 03:50)
-------------------------

1.10.4 (2013-08-28 03:13)
-------------------------

1.10.3 (2013-08-14)
-------------------

1.10.2 (2013-07-26)
-------------------

1.10.1 (2013-07-16)
-------------------

1.10.0 (2013-06-27)
-------------------

1.9.30 (2013-05-30)
-------------------

1.9.29 (2013-04-15)
-------------------

1.9.27 (2013-03-15 13:23)
-------------------------

1.9.26 (2013-03-15 10:38)
-------------------------

1.9.25 (2013-03-07)
-------------------

1.9.24 (2013-02-16)
-------------------

1.9.23 (2013-02-13)
-------------------

1.9.22 (2013-02-12 16:30)
-------------------------

1.9.21 (2013-02-12 14:00)
-------------------------

1.9.20 (2013-01-21)
-------------------

1.9.19 (2013-01-13)
-------------------

1.9.18 (2012-12-18)
-------------------

1.9.17 (2012-12-14)
-------------------

1.9.16 (2012-11-14 15:49)
-------------------------

1.9.15 (2012-11-13)
-------------------

1.9.14 (2012-11-14 02:20)
-------------------------

1.9.13 (2012-11-14 00:58)
-------------------------

1.9.12 (2012-11-06)
-------------------

1.9.11 (2012-11-02)
-------------------

1.9.10 (2012-11-01 11:10)
-------------------------

1.9.9 (2012-11-01 11:01)
------------------------

1.9.8 (2012-11-01 10:52)
------------------------

1.9.7 (2012-11-01 10:40)
------------------------

1.9.6 (2012-10-31)
------------------

1.9.5 (2012-10-19)
------------------

1.9.4 (2012-10-15 15:00)
------------------------

1.9.3 (2012-10-15 10:41)
------------------------

1.9.2 (2012-10-12 13:38)
------------------------

1.9.1 (2012-10-12 11:57)
------------------------

1.9.0 (2012-10-10)
------------------
