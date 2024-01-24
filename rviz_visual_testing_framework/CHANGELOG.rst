^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_visual_testing_framework
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

13.3.1 (2024-01-24)
-------------------

13.3.0 (2023-12-26)
-------------------

13.2.0 (2023-11-06)
-------------------

13.1.2 (2023-10-09)
-------------------

13.1.1 (2023-10-04)
-------------------

13.1.0 (2023-09-07)
-------------------

13.0.0 (2023-08-21)
-------------------

12.8.0 (2023-08-27)
-------------------

12.7.0 (2023-07-11)
-------------------
* Improve the compilation time of rviz_default_plugins (`#1007 <https://github.com/ros2/rviz/issues/1007>`_)
* Contributors: Chris Lalancette

12.6.1 (2023-06-12)
-------------------

12.6.0 (2023-06-07)
-------------------

12.5.1 (2023-05-11)
-------------------

12.5.0 (2023-04-28)
-------------------

12.4.0 (2023-04-18)
-------------------

12.3.2 (2023-04-11)
-------------------

12.3.1 (2023-03-01)
-------------------

12.3.0 (2023-02-14)
-------------------
* Update rviz to C++17. (`#939 <https://github.com/ros2/rviz/issues/939>`_)
* [rolling] Update maintainers - 2022-11-07 (`#923 <https://github.com/ros2/rviz/issues/923>`_)
* Contributors: Audrow Nash, Chris Lalancette

12.2.0 (2022-11-07)
-------------------

12.1.0 (2022-11-02)
-------------------

12.0.0 (2022-09-13)
-------------------
* Ogre 1.12.10 upgrade (`#878 <https://github.com/ros2/rviz/issues/878>`_)
* Contributors: Kenji Brameld

11.3.0 (2022-04-26)
-------------------

11.2.0 (2022-04-08)
-------------------

11.1.1 (2022-03-30)
-------------------

11.1.0 (2022-03-24)
-------------------

11.0.0 (2022-03-01)
-------------------

10.0.0 (2022-02-16)
-------------------
* Install headers to include/${PROJECT_NAME} (`#829 <https://github.com/ros2/rviz/issues/829>`_)
* Contributors: Shane Loretz

9.1.1 (2022-01-25)
------------------

9.1.0 (2022-01-13)
------------------

9.0.1 (2021-12-17)
------------------
* Fixes for uncrustify 0.72 (`#807 <https://github.com/ros2/rviz/issues/807>`_)
* Contributors: Chris Lalancette

9.0.0 (2021-11-18)
------------------

8.7.0 (2021-08-11)
------------------

8.6.0 (2021-05-13)
------------------
* Update includes after rcutils/get_env.h deprecation (`#677 <https://github.com/ros2/rviz/issues/677>`_)
* Contributors: Christophe Bedard

8.5.0 (2021-04-06)
------------------

8.4.0 (2021-03-18)
------------------
* Quiet a clang warning about a Qt memory leak. (`#651 <https://github.com/ros2/rviz/issues/651>`_)
* Contributors: Chris Lalancette

8.3.1 (2021-01-25)
------------------

8.3.0 (2020-12-08)
------------------
* use rcutils_get_env. (`#609 <https://github.com/ros2/rviz/issues/609>`_)
* Add linters and use ament_lint_auto (`#608 <https://github.com/ros2/rviz/issues/608>`_)
* Update maintainers (`#607 <https://github.com/ros2/rviz/issues/607>`_)
* Contributors: Jacob Perron, tomoya

8.2.0 (2020-06-23)
------------------

8.1.1 (2020-06-03)
------------------

8.1.0 (2020-06-03)
------------------
* Added missing virtual destructors (`#553 <https://github.com/ros2/rviz/issues/553>`_)
* Contributors: Ivan Santiago Paunovic

8.0.3 (2020-06-02)
------------------

8.0.2 (2020-05-21)
------------------
* Removed automoc completely. (`#545 <https://github.com/ros2/rviz/issues/545>`_)
* Contributors: Chris Lalancette

8.0.1 (2020-05-07)
------------------

8.0.0 (2020-05-01)
------------------
* Note from wjwwood: I've chosen bump the major version this time, even though the API was not broken strictly speaking, partly because of some potentially disruptive build system changes and partially in preparation for ROS Foxy, to allow for new minor/patch versions in the previous ROS release Eloquent.
* Changed to use ``ament_export_targets()``. (`#525 <https://github.com/ros2/rviz/issues/525>`_)
* Made some code style changes. (`#504 <https://github.com/ros2/rviz/issues/504>`_)
* Changed to install RViz configs for visual tests. (`#487 <https://github.com/ros2/rviz/issues/487>`_) (`#498 <https://github.com/ros2/rviz/issues/498>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas

7.0.3 (2019-11-13)
------------------

7.0.2 (2019-10-23)
------------------

7.0.1 (2019-10-04)
------------------

7.0.0 (2019-09-27)
------------------
* Fix typos in visual testing framework documentation (`#416 <https://github.com/ros2/rviz/issues/416>`_)
* Remove -Werror from defualt compiler options (`#420 <https://github.com/ros2/rviz/issues/420>`_)
* Contributors: Hunter L. Allen, Jacob Perron

6.1.1 (2019-05-29)
------------------

6.1.0 (2019-05-20)
------------------

6.0.0 (2019-05-08)
------------------

5.1.0 (2019-01-14)
------------------
* Migrate scalar displays, i.e. temperature, illuminance, relative humidity, and fluid pressure (`#367 <https://github.com/ros2/rviz/issues/367>`_)
* Contributors: GW1708

5.0.0 (2018-12-04)
------------------
* Made the transformation framework used by rviz pluggable. (`#346 <https://github.com/ros2/rviz/issues/346>`_)
* Increased visual testing stability (`#344 <https://github.com/ros2/rviz/issues/344>`_)
* Minor cleanup and fixes (`#336 <https://github.com/ros2/rviz/issues/336>`_)
  * Fix environment for colcon builds (no appends necessary)
  * Fix warning in visual_test_fixture.cpp and add -Werror option in CMakeLists.txt
  * Fix Qt moc warning for virtual signal
* Changed to support uncrustify 0.67 (`#333 <https://github.com/ros2/rviz/issues/333>`_)
  * fix closing block and tamplete indentation to comply with uncrustify 0.67
  * add space between reference and variable name
  * space before opening bracket
  * fix indend of inherited class
  * fix indent of code blocks
  * no space around -> operator
  * restore original spacing
* Contributors: Andreas Greimel, Martin Idel, Mikael Arguedas

4.0.1 (2018-06-28)
------------------
* Add Qt dependency to testing packages. (`#330 <https://github.com/ros2/rviz/issues/330>`_)
* Contributors: Steven! Ragnarök

4.0.0 (2018-06-27)
------------------
* Fixed some failing tests. (`#299 <https://github.com/ros2/rviz/issues/299>`_)
  * Fixed moved include files.
  * Fixed problem with TF visual test on Windows.
* Finished point cloud refactoring and testing. (`#250 <https://github.com/ros2/rviz/issues/250>`_)
* Improved visual testing framework and added more visual tests. (`#259 <https://github.com/ros2/rviz/issues/259>`_)
* Generalized the visual testing post build command (`#244 <https://github.com/ros2/rviz/issues/244>`_)
* Introduced visual testing framework for rviz. (`#209 <https://github.com/ros2/rviz/issues/209>`_)
* Contributors: Alessandro Bottero, Andreas Greimel, Martin Idel
