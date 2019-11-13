^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_rendering
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

7.0.3 (2019-11-13)
------------------

7.0.2 (2019-10-23)
------------------

7.0.1 (2019-10-04)
------------------
* Migrate Axes Display (`#429 <https://github.com/ros2/rviz/issues/429>`_)
* Contributors: Martin Idel

7.0.0 (2019-09-27)
------------------
* Fix map after upgrade (`#459 <https://github.com/ros2/rviz/issues/459>`_)
* Use eigen3_cmake_module (`#441 <https://github.com/ros2/rviz/issues/441>`_)
* Upgrade from Ogre 1.10 to Ogre 1.12.1 (`#394 <https://github.com/ros2/rviz/issues/394>`_)
* Remove -Werror from defualt compiler options (`#420 <https://github.com/ros2/rviz/issues/420>`_)
* Migrate Wrench Display (`#396 <https://github.com/ros2/rviz/issues/396>`_)
* Fix STL loader (`#410 <https://github.com/ros2/rviz/issues/410>`_)
* Contributors: Hunter L. Allen, Martin Idel, Shane Loretz

6.1.1 (2019-05-29)
------------------
* Fixed the installation destination of OGRE media resources which contain a ``dirname`` (`#404 <https://github.com/ros2/rviz/issues/404>`_)
* Contributors: Dirk Thomas

6.1.0 (2019-05-20)
------------------

6.0.0 (2019-05-08)
------------------

5.1.0 (2019-01-14)
------------------
* Handle FindEigen3 module's differing definitions (`#370 <https://github.com/ros2/rviz/issues/370>`_)
* Contributors: Scott K Logan

5.0.0 (2018-12-04)
------------------
* Export libraries to trigger hooks. (`#358 <https://github.com/ros2/rviz/issues/358>`_)
* Added virtual destructors for WindowManagerInterface and ViewportProjectionFinder (`#356 <https://github.com/ros2/rviz/issues/356>`_)
* Increased visual testing stability (`#344 <https://github.com/ros2/rviz/issues/344>`_)
* Cleanup dependencies for rviz_rendering_tests (`#326 <https://github.com/ros2/rviz/issues/326>`_)
* Changed to always build all tests and skip execution if not supported (`#342 <https://github.com/ros2/rviz/issues/342>`_)
* Changed to support uncrustify 0.67 (`#333 <https://github.com/ros2/rviz/issues/333>`_)
  * fix closing block and tamplete indentation to comply with uncrustify 0.67
  * add space between reference and variable name
  * space before opening bracket
  * fix indend of inherited class
  * fix indent of code blocks
  * no space around -> operator
  * restore original spacing
* Contributors: Andreas Greimel, Kartik Mohta, Mikael Arguedas, Steven! Ragnarök, William Woodall

4.0.1 (2018-06-28)
------------------
* move find_package or test dependencies to test block (`#331 <https://github.com/ros2/rviz/issues/331>`_)
* Contributors: William Woodall

4.0.0 (2018-06-27)
------------------
* Fixed the dependecy name for ``eigen``. (`#324 <https://github.com/ros2/rviz/issues/324>`_)
* Fixed bugs causing rviz to crash on macOS. (`#319 <https://github.com/ros2/rviz/issues/319>`_)
  * Fix Ogre assertion failure on Mac on resizing an Image or Camera display render window.
  * Fix segfault on Mac when resizing window after a Camera or Image Display was removed.
* Introduced visual testing framework for rviz. (`#209 <https://github.com/ros2/rviz/issues/209>`_)
* Restored the use of icons throughout rviz. (`#235 <https://github.com/ros2/rviz/issues/235>`_)
* Migrated the Path display. (`#236 <https://github.com/ros2/rviz/issues/236>`_)
* Migrated the marker display. (`#229 <https://github.com/ros2/rviz/issues/229>`_)
* Migrated RobotModel display. (`#210 <https://github.com/ros2/rviz/issues/210>`_)
* Changed to allow Ogre to delete its own render windows. (`#195 <https://github.com/ros2/rviz/issues/195>`_)
* Fixed compilation errors and runtime issues on Windows. (`#175 <https://github.com/ros2/rviz/issues/175>`_)
* Fixed a memory leak. (`#173 <https://github.com/ros2/rviz/issues/173>`_)
  * Signed-off-by: Chris Ye <chris.ye@intel.com>
* Refactored the Grid display. (`#165 <https://github.com/ros2/rviz/issues/165>`_)
* Remove now obsolete function. (`#163 <https://github.com/ros2/rviz/issues/163>`_)
  * It was made obsolete by pr `#136 <https://github.com/ros2/rviz/issues/136>`_ which removed the memcopy.
* Contributors: Alessandro Bottero, Andreas Greimel, Andreas Holzner, Dirk Thomas, Martin Idel, Mikael Arguedas, Steven! Ragnarök, William Woodall

3.0.0 (2018-02-07)
------------------
* Numerous changes to support Windows.
* Disabled anti-aliasing on Windows. (`#199 <https://github.com/ros2/rviz/issues/199>`_)
  * This fixes rendering issues on Windows when opening two or more render windows.
* Contributors: William Woodall

2.0.0 (2017-12-08)
------------------
* First version for ROS 2.
* Contributors: Andreas Greimel, Andreas Holzner, Hunter Allen, Johannes Jeising, Martin Idel, Steven! Ragnarok, William Woodall

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
