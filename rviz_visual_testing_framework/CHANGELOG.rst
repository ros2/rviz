^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_visual_testing_framework
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
