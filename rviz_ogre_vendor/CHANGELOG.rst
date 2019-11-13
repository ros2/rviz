^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_ogre_vendor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

7.0.3 (2019-11-13)
------------------
* Skip freetype2 dependencies (`#405 <https://github.com/ros2/rviz/issues/405>`_)
* Contributors: Sean Yen

7.0.2 (2019-10-23)
------------------
* Switch back to patch instead of git apply (`#470 <https://github.com/ros2/rviz/issues/470>`_)
* Remove OGRE_BUILD_COMPONENT_SAMPLES cmake arg.
* Contributors: Chris Lalancette

7.0.1 (2019-10-04)
------------------
* Fix the rviz_ogre_vendor packaging.
* Contributors: Chris Lalancette

7.0.0 (2019-09-27)
------------------
* Add .dsv file beside custom environment hook (`#449 <https://github.com/ros2/rviz/issues/449>`_)
* Upgrade from Ogre 1.10 to Ogre 1.12.1 (`#394 <https://github.com/ros2/rviz/issues/394>`_)
* Mojave compatibility (`#414 <https://github.com/ros2/rviz/issues/414>`_)
* Contributors: Dirk Thomas, Karsten Knese, Martin Idel

6.1.1 (2019-05-29)
------------------

6.1.0 (2019-05-20)
------------------
* Upgraded to OGRE 1.10.12 to get a macOS fix but also not break any APIs by upgrading to OGRE 1.11.x. (`#380 <https://github.com/ros2/rviz/issues/380>`_)
* Contributors: Emerson Knapp

6.0.0 (2019-05-08)
------------------
* Suppress ogre_vendor warnings in clang+libcxx build. The -w flag was not strong enough for Clang builds. (`#389 <https://github.com/ros2/rviz/issues/389>`_)
  Signed-off-by: Emerson Knapp <eknapp@amazon.com>
* Pass through only the stdlib flag to the vendor build, instead of all C++ flags (`#388 <https://github.com/ros2/rviz/issues/388>`_)
  Signed-off-by: Emerson Knapp <eknapp@amazon.com>
* Pass through CXX flags to OGRE vendor build (`#381 <https://github.com/ros2/rviz/issues/381>`_)
  * Pass through CXX flags
  Signed-off-by: Emerson Knapp <eknapp@amazon.com>
  * fixup
  Signed-off-by: William Woodall <william@osrfoundation.org>
  * re-add removed libc++ flag, because OSX build always needs it
  Signed-off-by: Emerson Knapp <eknapp@amazon.com>
* Propagate toolchain-file to external-project (`#374 <https://github.com/ros2/rviz/issues/374>`_)
  If defined, propagate the CMAKE_TOOLCHAIN_FILE argument to the cmake
  argument of freetype, zlib and ogre projects.
  Change-Id: Ibf2802b96c2238a06191e78a1b2a3128769a83af
  Signed-off-by: Louis Mayencourt <louis.mayencourt@arm.com>
* Contributors: Emerson Knapp, lmayencourt

5.1.0 (2019-01-14)
------------------
* Skip the system directories when looking for OGRE (`#371 <https://github.com/ros2/rviz/issues/371>`_)
* Contributors: Scott K Logan

5.0.0 (2018-12-04)
------------------
* Changed ZLIB_ROOT -> ZLIB_DIR (`#349 <https://github.com/ros2/rviz/issues/349>`_)
* Contributors: Mikael Arguedas

4.0.1 (2018-06-28)
------------------

4.0.0 (2018-06-27)
------------------
* Changed the download timeout for Ogre to be twenty (20) minutes. (`#323 <https://github.com/ros2/rviz/issues/323>`_)
* Contributors: Dirk Thomas, Martin Idel, Mikael Arguedas, Russ

3.0.0 (2018-02-07)
------------------
* Updated Ogre to 1.10.11. (`#181 <https://github.com/ros2/rviz/issues/181>`_)

2.0.0 (2017-12-08)
------------------
* First version for ROS 2.
* Contributors: Andreas Greimel, Andreas Holzner, Chris Ye, Johannes Jeising, Martin Idel, Steven! Ragnarok, William Woodall

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
