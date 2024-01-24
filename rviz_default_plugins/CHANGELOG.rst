^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_default_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

13.3.1 (2024-01-24)
-------------------
* Fix time-syncing message (`#1121 <https://github.com/ros2/rviz/issues/1121>`_)
* Switch from ROS_TIME to SYSTEM_TIME on rclcpp::Time construction (`#1117 <https://github.com/ros2/rviz/issues/1117>`_)
* Append measured subscription frequency to topic status (`#1113 <https://github.com/ros2/rviz/issues/1113>`_)
* Contributors: Alejandro Hernández Cordero, Austin Moore, Yadu

13.3.0 (2023-12-26)
-------------------
* Fix typo (`#1104 <https://github.com/ros2/rviz/issues/1104>`_)
* Fix potencial leak / seg fault (`#726 <https://github.com/ros2/rviz/issues/726>`_)
* Fixed screw display (`#1093 <https://github.com/ros2/rviz/issues/1093>`_)
* Explicit time conversions and comparisons (`#1087 <https://github.com/ros2/rviz/issues/1087>`_)
* Handle missing effort limit in URDF (`#1084 <https://github.com/ros2/rviz/issues/1084>`_)
* Contributors: AiVerisimilitude, Alejandro Hernández Cordero, Christoph Fröhlich, Patrick Roncagliolo

13.2.0 (2023-11-06)
-------------------
* (robot) fix styling of log msg (`#1080 <https://github.com/ros2/rviz/issues/1080>`_)
* Fix image display wrapping (`#1038 <https://github.com/ros2/rviz/issues/1038>`_)
* removed enableInteraction reference (`#1075 <https://github.com/ros2/rviz/issues/1075>`_)
* Contributors: Alejandro Hernández Cordero, Lewe Christiansen, Matthijs van der Burgh

13.1.2 (2023-10-09)
-------------------
* Fix ODR violations in interactive_marker displays. (`#1068 <https://github.com/ros2/rviz/issues/1068>`_)
* Contributors: Chris Lalancette

13.1.1 (2023-10-04)
-------------------

13.1.0 (2023-09-07)
-------------------
* Improve error handling in LaserScanDisplay (`#1035 <https://github.com/ros2/rviz/issues/1035>`_)
* Fix implicit capture of "this" warning in C++20 (`#1053 <https://github.com/ros2/rviz/issues/1053>`_)
* Removed unused code (`#1044 <https://github.com/ros2/rviz/issues/1044>`_)
* Contributors: AiVerisimilitude, Alejandro Hernández Cordero

13.0.0 (2023-08-21)
-------------------
* Fixed AccelStamped, TwistStamped and Wrench icons (`#1041 <https://github.com/ros2/rviz/issues/1041>`_)
* Fix the flakey rviz_rendering tests (`#1026 <https://github.com/ros2/rviz/issues/1026>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette

12.8.0 (2023-08-27)
-------------------
* Don't pass screw_display.hpp to the moc generator. (`#1018 <https://github.com/ros2/rviz/issues/1018>`_)
  Since it isn't a Qt class, you get a warning from moc:
  Note: No relevant classes found. No output generated.
  Just skip adding it to the moc list here, which gets rid
  of the warning.
* Added DepthCloud default plugin (`#996 <https://github.com/ros2/rviz/issues/996>`_)
* Added TwistStamped and AccelStamped default plugins (`#991 <https://github.com/ros2/rviz/issues/991>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette

12.7.0 (2023-07-11)
-------------------
* Added Effort plugin (`#990 <https://github.com/ros2/rviz/issues/990>`_)
* Improve the compilation time of rviz_default_plugins (`#1007 <https://github.com/ros2/rviz/issues/1007>`_)
* Switch to ament_cmake_vendor_package (`#995 <https://github.com/ros2/rviz/issues/995>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Scott K Logan

12.6.1 (2023-06-12)
-------------------

12.6.0 (2023-06-07)
-------------------
* Modify access specifier to protected or public for the scope of processMessage() member function (`#984 <https://github.com/ros2/rviz/issues/984>`_)
* Contributors: Hyunseok

12.5.1 (2023-05-11)
-------------------

12.5.0 (2023-04-28)
-------------------

12.4.0 (2023-04-18)
-------------------

12.3.2 (2023-04-11)
-------------------
* Fix ODR errors with gmock (`#967 <https://github.com/ros2/rviz/issues/967>`_)
* Update Frame shortcut (`#958 <https://github.com/ros2/rviz/issues/958>`_)
* Contributors: David V. Lu!!, methylDragon

12.3.1 (2023-03-01)
-------------------
* point_marker: fix bug where the number of rendered points accumulates over time (`#949 <https://github.com/ros2/rviz/issues/949>`_)
* Contributors: AndreasR30

12.3.0 (2023-02-14)
-------------------
* Update rviz to C++17. (`#939 <https://github.com/ros2/rviz/issues/939>`_)
* Fix tolerance calculation precision (`#934 <https://github.com/ros2/rviz/issues/934>`_)
* Fix MeshResourceMarker for mesh with color-based embedded material (`#928 <https://github.com/ros2/rviz/issues/928>`_)
* [rolling] Update maintainers - 2022-11-07 (`#923 <https://github.com/ros2/rviz/issues/923>`_)
* Contributors: Audrow Nash, Chris Lalancette, Xavier BROQUERE, Xenofon Karamanos

12.2.0 (2022-11-07)
-------------------

12.1.0 (2022-11-02)
-------------------
* Add Map Display binary option (`#846 <https://github.com/ros2/rviz/issues/846>`_)
* Delete frame_locked_markers when reusing marker (`#907 <https://github.com/ros2/rviz/issues/907>`_)
* Consider region of interest in CameraDisplay (`#864 <https://github.com/ros2/rviz/issues/864>`_)
* std::copy fix - OccupancyGridUpdate - Data is not being processed correctly (`#895 <https://github.com/ros2/rviz/issues/895>`_)
* Contributors: AndreasR30, Eric, Patrick Roncagliolo, Shane Loretz

12.0.0 (2022-09-13)
-------------------
* Set error status when duplicate markers are in the same MarkerArray (`#891 <https://github.com/ros2/rviz/issues/891>`_)
* Make Axes display use latest transform (`#892 <https://github.com/ros2/rviz/issues/892>`_)
* Show link names in inertia error message (`#874 <https://github.com/ros2/rviz/issues/874>`_)
* Ogre 1.12.10 upgrade (`#878 <https://github.com/ros2/rviz/issues/878>`_)
* Use make_shared to construct PointCloud2 (`#869 <https://github.com/ros2/rviz/issues/869>`_)
* Fix include order (`#858 <https://github.com/ros2/rviz/issues/858>`_)
* Contributors: Hunter L. Allen, Jacob Perron, Kenji Brameld, Shane Loretz, Timon Engelke

11.3.0 (2022-04-26)
-------------------

11.2.0 (2022-04-08)
-------------------
* Add far plane distance property to camera (`#849 <https://github.com/ros2/rviz/issues/849>`_)
* Contributors: Paul

11.1.1 (2022-03-30)
-------------------

11.1.0 (2022-03-24)
-------------------

11.0.0 (2022-03-01)
-------------------
* Drop ignition-math6 from rviz_default_plugins link interface (`#833 <https://github.com/ros2/rviz/issues/833>`_)
* add implementation for cancel interface (`#809 <https://github.com/ros2/rviz/issues/809>`_)
* Contributors: Chen Lihui, Scott K Logan

10.0.0 (2022-02-16)
-------------------
* Install headers to include/${PROJECT_NAME} (`#829 <https://github.com/ros2/rviz/issues/829>`_)
* Remove definition of PLUGINLIB_DISABLE_BOOST. (`#821 <https://github.com/ros2/rviz/issues/821>`_)
* Contributors: Chris Lalancette, Shane Loretz

9.1.1 (2022-01-25)
------------------

9.1.0 (2022-01-13)
------------------
* Remove TF filter from ImageTransportDisplay (`#788 <https://github.com/ros2/rviz/issues/788>`_)
* Add underscores to material names (`#811 <https://github.com/ros2/rviz/issues/811>`_)
* Export image_transport dependency (`#813 <https://github.com/ros2/rviz/issues/813>`_)
* Contributors: Chen Lihui, Cory Crean, Jacob Perron

9.0.1 (2021-12-17)
------------------
* Fixes for uncrustify 0.72 (`#807 <https://github.com/ros2/rviz/issues/807>`_)
* Contributors: Chris Lalancette

9.0.0 (2021-11-18)
------------------
* Switch to using Qt::MiddleButton for RViz. (`#802 <https://github.com/ros2/rviz/issues/802>`_)
* Add a tf_buffer_cache_time_ns to tf_wrapper (`#792 <https://github.com/ros2/rviz/issues/792>`_)
* Make libraries to avoid compiling files multiple times (`#774 <https://github.com/ros2/rviz/issues/774>`_)
* Computed inertia with ignition-math (`#751 <https://github.com/ros2/rviz/issues/751>`_)
* Fixed crash when changing rendering parameters for pointcloud2 while 'Selectable' box is unchecked (`#768 <https://github.com/ros2/rviz/issues/768>`_)
* Robot: Report mesh loading issues (`#744 <https://github.com/ros2/rviz/issues/744>`_)
* Handle NaN values for Wrench msgs (`#746 <https://github.com/ros2/rviz/issues/746>`_)
* Triangle lists support textures (`#719 <https://github.com/ros2/rviz/issues/719>`_)
* Report sample lost events (`#686 <https://github.com/ros2/rviz/issues/686>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Gonzo, Greg Balke, Ivan Santiago Paunovic, Shane Loretz, bailaC

8.7.0 (2021-08-11)
------------------
* Fix path message orientation error (`#736 <https://github.com/ros2/rviz/issues/736>`_)
* Set topic namespace in interactive markers display (`#725 <https://github.com/ros2/rviz/issues/725>`_)
* mass property visualization (`#714 <https://github.com/ros2/rviz/issues/714>`_)
* Export InteractiveMarker (`#718 <https://github.com/ros2/rviz/issues/718>`_)
* Yuv to rgb changes (`#701 <https://github.com/ros2/rviz/issues/701>`_)
* Extract message type in ImageTransportDisplay (`#711 <https://github.com/ros2/rviz/issues/711>`_)
* Duplicated code RobotJoint (`#702 <https://github.com/ros2/rviz/issues/702>`_)
* Don't attempt to moc generate files that don't have QOBJECT. (`#690 <https://github.com/ros2/rviz/issues/690>`_)
* Switch to including tf2_geometry_msgs.hpp (`#689 <https://github.com/ros2/rviz/issues/689>`_)
* Contributors: Akash, Alejandro Hernández Cordero, Chris Lalancette, Paul, Rebecca Butler, brian soe, cturcotte-qnx

8.6.0 (2021-05-13)
------------------
* Export Qt5 dependencies properly (`#687 <https://github.com/ros2/rviz/issues/687>`_)
* Add support for namespace-scoped DELETEALL action in Marker displays (`#685 <https://github.com/ros2/rviz/issues/685>`_)
* Use image_transport to subscribe to image messages (`#523 <https://github.com/ros2/rviz/issues/523>`_)
* Contributors: Audrow Nash, ketatam, Martin Idel, Michel Hidalgo

8.5.0 (2021-04-06)
------------------

8.4.0 (2021-03-18)
------------------
* Add ViewPicker::get3DPatch to the public API (`#657 <https://github.com/ros2/rviz/issues/657>`_)
* Allow to zoom more with orbit controller (`#654 <https://github.com/ros2/rviz/issues/654>`_)
* Contributors: Joseph Schornak, Victor Lamoine

8.3.1 (2021-01-25)
------------------
* Fix possible nullptr access in robot_joint.cpp. (`#636 <https://github.com/ros2/rviz/issues/636>`_)
* Contributors: Chris Lalancette

8.3.0 (2020-12-08)
------------------
* Fix for mousewheel to zoom in/out (`#623 <https://github.com/ros2/rviz/issues/623>`_)
* Make the types explicit in quaternion_helper.hpp. (`#625 <https://github.com/ros2/rviz/issues/625>`_)
* Update status message by removing colon or adjust colon position (`#624 <https://github.com/ros2/rviz/issues/624>`_)
* Do not use assume every RenderPanel has a ViewController. (`#613 <https://github.com/ros2/rviz/issues/613>`_)
* Add linters and use ament_lint_auto (`#608 <https://github.com/ros2/rviz/issues/608>`_)
* Update maintainers (`#607 <https://github.com/ros2/rviz/issues/607>`_)
* TimePanel port (`#599 <https://github.com/ros2/rviz/issues/599>`_)
* Upgrade to tinyxml2 for rviz (`#418 <https://github.com/ros2/rviz/issues/418>`_)
* Use retriever.hpp (`#589 <https://github.com/ros2/rviz/issues/589>`_)
* Added covariance settings to set pose estimate (`#569 <https://github.com/ros2/rviz/issues/569>`_)
* use reference in range loops to avoid copy (`#577 <https://github.com/ros2/rviz/issues/577>`_)
* Contributors: Chen Lihui, Chris Lalancette, Dirk Thomas, Jacob Perron, Martin Idel, Matthijs den Toom, Michel Hidalgo, Nico Neumann, Shane Loretz

8.2.0 (2020-06-23)
------------------
* Changed to not install test header files in rviz_rendering. (`#564 <https://github.com/ros2/rviz/issues/564>`_)
* Changed to use a dedicated TransformListener thread. (`#551 <https://github.com/ros2/rviz/issues/551>`_)
* Suppressed warnings when building with older Qt versions. (`#562 <https://github.com/ros2/rviz/issues/562>`_)
* Restored compatibility with older Qt versions (`#561 <https://github.com/ros2/rviz/issues/561>`_)
* Contributors: Chris Lalancette, Dirk Thomas, ymd-stella

8.1.1 (2020-06-03)
------------------
* avoid absolute OGRE path in exported targets (`#558 <https://github.com/ros2/rviz/issues/558>`_)
* Contributors: Dirk Thomas

8.1.0 (2020-06-03)
------------------
* Added missing virtual destructors (`#553 <https://github.com/ros2/rviz/issues/553>`_)
* Contributors: Ivan Santiago Paunovic

8.0.3 (2020-06-02)
------------------
* Fixed deprecated Qt usage. (`#555 <https://github.com/ros2/rviz/issues/555>`_)
* Contributors: Jacob Perron

8.0.2 (2020-05-21)
------------------
* Removed rviz_default_plugins dependency on TinyXML (`#531 <https://github.com/ros2/rviz/issues/531>`_)
  This clears the way for urdf to switch to TinyXML2
  Note that internally, urdf was converting the passed XML to a string and reparsing it in the implementation of ``urdf::model::initXml``
* Contributors: Dan Rose

8.0.1 (2020-05-07)
------------------
* Added dependency on ogre to fix building on the buildfarm (`#544 <https://github.com/ros2/rviz/issues/544>`_)
* Refactored test fixtures to reduce memory usage while compiling (`#540 <https://github.com/ros2/rviz/pull/540>`_)
* Contributors: Chris Lalancette

8.0.0 (2020-05-01)
------------------
* Note from wjwwood: I've chosen bump the major version this time, even though the API was not broken strictly speaking, partly because of some potentially disruptive build system changes and partially in preparation for ROS Foxy, to allow for new minor/patch versions in the previous ROS release Eloquent.
* Removed duplicate include dirs and link libraries. (`#533 <https://github.com/ros2/rviz/issues/533>`_)
* Updated includes to use non-entry point headers from detail subdir. (`#526 <https://github.com/ros2/rviz/issues/526>`_)
* Changed to use ``ament_export_targets()``. (`#525 <https://github.com/ros2/rviz/issues/525>`_)
* Changed to use the clock from the node in tools. (`#519 <https://github.com/ros2/rviz/issues/519>`_)
* Changed to allow the MapDisplay "Update Topic" to be changed. (`#517 <https://github.com/ros2/rviz/issues/517>`_)
  The major reason for this is so that the "Update Topic"
  (and more importantly the QoS profile) is saved when clicking
  "Save Config" in RViz2.  The more minor reason is that a user
  *might* want to use a different topic for this.  We still
  auto-populate this field with <topic_name>_updates by default,
  but the user can now override it.
* Made some code style changes. (`#504 <https://github.com/ros2/rviz/issues/504>`_)
* Fixed camera info for camera display. (`#419 <https://github.com/ros2/rviz/issues/419>`_)
* Fixed wrong resource group for robot links. (`#495 <https://github.com/ros2/rviz/issues/495>`_)
* Changed default goal to ``goal_pose`` and not just in default rviz. (`#491 <https://github.com/ros2/rviz/issues/491>`_)
* Fixed a bug by setting the clock type if Marker ``frame_locked`` is true. (`#482 <https://github.com/ros2/rviz/issues/482>`_)
  Fixes `#479 <https://github.com/ros2/rviz/issues/479>`_
* Fixed the map display for moving TF frame. (`#483 <https://github.com/ros2/rviz/issues/483>`_)
  Instead of the current time, use Time(0) to get the latest available transform as a fallback.
  This is the same logic that is applied in RViz from ROS 1.
  Resolves `#332 <https://github.com/ros2/rviz/issues/332>`_
* Migrated pose with covariance display. (`#471 <https://github.com/ros2/rviz/issues/471>`_)
* Fixed build when included as a sub-project. (`#475 <https://github.com/ros2/rviz/issues/475>`_)
* Added icon copyrights + PoseWithCovariance icon. (`#430 <https://github.com/ros2/rviz/issues/430>`_)
* Contributors: Chris Lalancette, Dan Rose, Dirk Thomas, Jacob Perron, Martin Idel, Michel Hidalgo, Steven Macenski, chapulina

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
