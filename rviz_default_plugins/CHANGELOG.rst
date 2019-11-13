^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_default_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

7.0.3 (2019-11-13)
------------------

7.0.2 (2019-10-23)
------------------
* Use clock from context in markers (`#472 <https://github.com/ros2/rviz/issues/472>`_)
* Contributors: Martin Idel

7.0.1 (2019-10-04)
------------------
* Migrate Axes Display (`#429 <https://github.com/ros2/rviz/issues/429>`_)
* Contributors: Martin Idel

7.0.0 (2019-09-27)
------------------
* Introduce QoS property (`#409 <https://github.com/ros2/rviz/issues/409>`_)
  A container of properties related to QoS settings. Replaces queue size and unreliable properties.
* Rename interactive marker client enum (`#465 <https://github.com/ros2/rviz/issues/465>`_)
* Migrate InteractiveMarkerDisplay (`#457 <https://github.com/ros2/rviz/issues/457>`_)
* Fix map after upgrade (`#459 <https://github.com/ros2/rviz/issues/459>`_)
* Rename 2d Nav Goal to 2d Goal Pose (`#455 <https://github.com/ros2/rviz/issues/455>`_)
    * Rename nav pose tool to goal pose tool
    * Rename topic for goal pose tool from "move_base_simple/goal" to "goal_pose"
* Do not select interactive markers when mousing over them (`#451 <https://github.com/ros2/rviz/issues/451>`_)
* Migrate Interaction Tool (`#423 <https://github.com/ros2/rviz/issues/423>`_)
* Upgrade from Ogre 1.10 to Ogre 1.12.1 (`#394 <https://github.com/ros2/rviz/issues/394>`_)
* Re-enable use of tf message filter (`#375 <https://github.com/ros2/rviz/issues/375>`_)
* Fix map display (`#425 <https://github.com/ros2/rviz/issues/425>`_)
* FrameTransformer implements tf2::BufferCoreInterface and tf2_ros::AsyncBufferInterface (`#422 <https://github.com/ros2/rviz/issues/422>`_)
* Disambiguate "estimate" pose from "goal" pose in log (`#427 <https://github.com/ros2/rviz/issues/427>`_)
* Mojave compatibility (`#414 <https://github.com/ros2/rviz/issues/414>`_)
* Use geometry_msgs types instead of custom types (`#426 <https://github.com/ros2/rviz/issues/426>`_)
* Remove -Werror from defualt compiler options (`#420 <https://github.com/ros2/rviz/issues/420>`_)
* Migrate Wrench Display (`#396 <https://github.com/ros2/rviz/issues/396>`_)
* Contributors: Dan Rose, Hunter L. Allen, Jacob Perron, Karsten Knese, Martin Idel, Shivesh Khaitan, Steven Macenski

6.1.1 (2019-05-29)
------------------

6.1.0 (2019-05-20)
------------------
* Updated to use the new specification for types from the ROS node graph API. (`#387 <https://github.com/ros2/rviz/issues/387>`_)
* Contributors: Jacob Perron

6.0.0 (2019-05-08)
------------------
* Made changes to avoid newly deprecated API's related to publishers and subscriptions. (`#399 <https://github.com/ros2/rviz/issues/399>`_)
* Made changes to avoid newly deprecated API's related to publish calls that used ``shared_ptr``. signature (`#398 <https://github.com/ros2/rviz/issues/398>`_)
* Changed to use the ``ament_include_directories_order`` macro to ensure header include path ordering is correct. (`#384 <https://github.com/ros2/rviz/issues/384>`_)
* Made changes to fix interoperability with ``robot_state_publisher``. `#14 <https://github.com/ros2/rviz/issues/14>`_ (`#378 <https://github.com/ros2/rviz/issues/378>`_)
* Contributors: Karsten Knese, William Woodall, ivanpauno

5.1.0 (2019-01-14)
------------------
* Migrate scalar displays, i.e. temperature, illuminance, relative humidity, and fluid pressure (`#367 <https://github.com/ros2/rviz/issues/367>`_)
* Fix errors from uncrustify v0.68 (`#366 <https://github.com/ros2/rviz/issues/366>`_)
* Visibility followup for Swatch marker (`#297 <https://github.com/ros2/rviz/issues/297>`_)
* Contributors: GW1708, Jacob Perron, Martin Idel, William Woodall

5.0.0 (2018-12-04)
------------------
* Export libraries to trigger hooks. (`#358 <https://github.com/ros2/rviz/issues/358>`_)
* Made the transformation framework used by rviz pluggable. (`#346 <https://github.com/ros2/rviz/issues/346>`_)
* Added hotkeys (also to help) (`#312 <https://github.com/ros2/rviz/issues/312>`_)
* Migrated Range Display (`#325 <https://github.com/ros2/rviz/issues/325>`_)
* Migrated pose estimate tool (`#329 <https://github.com/ros2/rviz/issues/329>`_)
* Increased visual testing stability (`#344 <https://github.com/ros2/rviz/issues/344>`_)
* Fixed laserscan 1 meter limit bug (`#345 <https://github.com/ros2/rviz/issues/345>`_)
* Changed to always build all tests and skip execution if not supported (`#342 <https://github.com/ros2/rviz/issues/342>`_)
* Minor cleanup and fixes (`#336 <https://github.com/ros2/rviz/issues/336>`_)
  * Fix environment for colcon builds (no appends necessary)
  * Fix warning in visual_test_fixture.cpp and add -Werror option in CMakeLists.txt
  * Fix Qt moc warning for virtual signal
* Fixed missing status in laser scan display (`#335 <https://github.com/ros2/rviz/issues/335>`_)
  * Show status error when transform failed
* Changes due to uncrustify 0.67 (`#333 <https://github.com/ros2/rviz/issues/333>`_)
  * fix closing block and tamplete indentation to comply with uncrustify 0.67
  * add space between reference and variable name
  * space before opening bracket
  * fix indend of inherited class
  * fix indent of code blocks
  * no space around -> operator
  * restore original spacing
* Contributors: Alessandro Bottero, Andreas Greimel, Martin Idel, Mikael Arguedas, Steven! Ragnarök, eric1221bday

4.0.1 (2018-06-28)
------------------

4.0.0 (2018-06-27)
------------------
* Polished selection behavior. (`#315 <https://github.com/ros2/rviz/issues/315>`_)
* Suppressed a warning when disabling a map display. (`#320 <https://github.com/ros2/rviz/issues/320>`_)
* Fixed point cloud selection. (`#307 <https://github.com/ros2/rviz/issues/307>`_)
* Fixed a segfault for tf on Windows. (`#310 <https://github.com/ros2/rviz/issues/310>`_)
* Added a warning when the ``camera_info`` is missing. (`#311 <https://github.com/ros2/rviz/issues/311>`_)
* Polished tests and migrate or delete old tests. (`#289 <https://github.com/ros2/rviz/issues/289>`_)
* Migrated the map display. (`#267 <https://github.com/ros2/rviz/issues/267>`_)
* Migrated the Marker Array display. (`#296 <https://github.com/ros2/rviz/issues/296>`_)
* Migrated the 2D Nav Goal tool. (`#294 <https://github.com/ros2/rviz/issues/294>`_)
* Migrated the Third Person Follower View Controller. (`#295 <https://github.com/ros2/rviz/issues/295>`_)
* Implemented a workaround for the missing identity transform when transforming from a frame to itself. (`#298 <https://github.com/ros2/rviz/issues/298>`_)
* Migrated the GridCells display. (`#286 <https://github.com/ros2/rviz/issues/286>`_)
* Migrated the fps camera view controller. (`#281 <https://github.com/ros2/rviz/issues/281>`_)
* Migrated the point stamped display. (`#278 <https://github.com/ros2/rviz/issues/278>`_)
* Fixed the splash screen and the help panel. (`#277 <https://github.com/ros2/rviz/issues/277>`_)
* Migrated the odometry display. (`#275 <https://github.com/ros2/rviz/issues/275>`_)
* Migrated the focus tool. (`#266 <https://github.com/ros2/rviz/issues/266>`_)
* Homogenized behavior of rviz when a tf transform fails. (`#292 <https://github.com/ros2/rviz/issues/292>`_)
* Updated rviz to only use a single ros node. (`#197 <https://github.com/ros2/rviz/issues/197>`_)
* Migrated XYOrbit View Controller. (`#282 <https://github.com/ros2/rviz/issues/282>`_)
* Fixed a segfault in the selection manager tests. (`#284 <https://github.com/ros2/rviz/issues/284>`_)
* Fixed some of the ``plugins_description`` entities. (`#285 <https://github.com/ros2/rviz/issues/285>`_)
* Migrated the orthographic view controller. (`#270 <https://github.com/ros2/rviz/issues/270>`_)
* Migrated the measure tool. (`#264 <https://github.com/ros2/rviz/issues/264>`_)
* Migrated the publish point tool. (`#262 <https://github.com/ros2/rviz/issues/262>`_)
* Added message type to MarkerDisplay plugin description. (`#252 <https://github.com/ros2/rviz/issues/252>`_)
* Finished point cloud refactoring and testing. (`#250 <https://github.com/ros2/rviz/issues/250>`_)
* Improved visual testing framework and added visual tests. (`#259 <https://github.com/ros2/rviz/issues/259>`_)
* Fixed debug build on macOS. (`#258 <https://github.com/ros2/rviz/issues/258>`_)
* Fixed a build error for ignored qualifiers warning/error on GCC8. (`#261 <https://github.com/ros2/rviz/issues/261>`_)
* Migrated the select tool. (`#256 <https://github.com/ros2/rviz/issues/256>`_)
* Disabled Ogre deprecation warnings on Windows. (`#242 <https://github.com/ros2/rviz/issues/242>`_)
* Introduced visual testing framework for rviz. (`#209 <https://github.com/ros2/rviz/issues/209>`_)
* Fixed "display" tests and enable when the test environment allows. (`#186 <https://github.com/ros2/rviz/issues/186>`_)
* Migrated the LaserScan display. (`#238 <https://github.com/ros2/rviz/issues/238>`_)
* Restored the use of icons throughout rviz. (`#235 <https://github.com/ros2/rviz/issues/235>`_)
* Migrated the Path display. (`#236 <https://github.com/ros2/rviz/issues/236>`_)
* Migrated the pose array display. (`#233 <https://github.com/ros2/rviz/issues/233>`_)
* Migrated the marker display. (`#229 <https://github.com/ros2/rviz/issues/229>`_)
* Migrated the Pose display. (`#204 <https://github.com/ros2/rviz/issues/204>`_)
* Changed the default position of the camera in the RenderPanel. (`#205 <https://github.com/ros2/rviz/issues/205>`_)
* Migrated the RobotModel display. (`#210 <https://github.com/ros2/rviz/issues/210>`_)
* Fixed warnings from pluginlib. (`#196 <https://github.com/ros2/rviz/issues/196>`_)
* Fixed a bug which caused rviz to crash when removing a display. (`#191 <https://github.com/ros2/rviz/issues/191>`_)
* Refactored the Grid display. (`#165 <https://github.com/ros2/rviz/issues/165>`_)
* Contributors: Alessandro Bottero, Andreas Greimel, Andreas Holzner, Dirk Thomas, Martin Idel, Mikael Arguedas, Steven! Ragnarök, William Woodall, mjbogusz

3.0.0 (2018-02-07)
------------------
* Fixed compilation errors and runtime issues on Windows. (`#175 <https://github.com/ros2/rviz/issues/175>`_)
* Migrated the camera display. (`#183 <https://github.com/ros2/rviz/issues/183>`_)
* Migrated the tf display. (`#182 <https://github.com/ros2/rviz/issues/182>`_)
* Migrated the Image display. (`#164 <https://github.com/ros2/rviz/issues/164>`_)
* Migrated code for point cloud displays to ``rviz_default_plugins``. (`#153 <https://github.com/ros2/rviz/issues/153>`_)
* Fixed a bug where the PointCloud2 display was not accepting valid points. (`#189 <https://github.com/ros2/rviz/issues/189>`_)
* Migrated the polygon display. (`#194 <https://github.com/ros2/rviz/issues/194>`_)
* Contributors: William Woodall

2.0.0 (2017-12-08)
------------------
* First version for ROS 2.
* Contributors: Steven! Ragnarok, William Woodall

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
