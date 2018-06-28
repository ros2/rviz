^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_rendering
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.1 (2018-06-27)
------------------
* fix dependecy name for eigen (`#324 <https://github.com/ros2/rviz/issues/324>`_)
* Contributors: William Woodall

2.1.0 (2018-06-27)
------------------
* Fixed bugs causing rviz to crash on macOS. (`#319 <https://github.com/ros2/rviz/issues/319>`_)
  * Fix Ogre assertion failure on Mac on resizing an Image or Camera display render window.
  * Fix segfault on Mac when resizing window after a Camera or Image Display was removed.
* Introduced visual testing framework for rviz. (`#209 <https://github.com/ros2/rviz/issues/209>`_)
* Restored the use of icons throughout rviz. (`#235 <https://github.com/ros2/rviz/issues/235>`_)
* Migrated the Path display. (`#236 <https://github.com/ros2/rviz/issues/236>`_)
* Migrated the marker display. (`#229 <https://github.com/ros2/rviz/issues/229>`_)
* Migrated RobotModel display. (`#210 <https://github.com/ros2/rviz/issues/210>`_)
* Disabled anti-aliasing on Windows. (`#199 <https://github.com/ros2/rviz/issues/199>`_)
  * This fixes rendering issues on Windows when opening two or more render windows.
* Changed to allow Ogre to delete its own render windows. (`#195 <https://github.com/ros2/rviz/issues/195>`_)
* Fixed compilation errors and runtime issues on Windows. (`#175 <https://github.com/ros2/rviz/issues/175>`_)
* Fixed a memory leak. (`#173 <https://github.com/ros2/rviz/issues/173>`_)
  * Signed-off-by: Chris Ye <chris.ye@intel.com>
* Refactored the Grid display. (`#165 <https://github.com/ros2/rviz/issues/165>`_)
* Remove now obsolete function. (`#163 <https://github.com/ros2/rviz/issues/163>`_)
  * It was made obsolete by pr `#136 <https://github.com/ros2/rviz/issues/136>`_ which removed the memcopy.
* Contributors: Alessandro Bottero, Andreas Greimel, Andreas Holzner, Dirk Thomas, Martin Idel, Mikael Arguedas, Steven! Ragnarök, William Woodall
