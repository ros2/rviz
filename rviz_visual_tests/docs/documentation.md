# RViz 3D Visual Testing Framework

This is a testing framework which allows the user to take screenshots of the main 3D rendered 
scene and possibly of other secondary render windows, and to compare them with previously captured 
reference screenshots.


## Assumptions and Settings

- Visual tests are located in the folder rviz_visual_tests.

- The reference screenshots are located in the rviz_visual_tests/tests source folder, in a
directory called reference_images. The test images are, instead, located in the rviz_visual_tests
build folder, in a directory called test_images, generated automatically at build system generation.

- Visual tests are especially useful for regression testing.

- The user will be able to do both of the following: either take/update reference images, or take
test images and compare them with existing references.

- By default the comparison will be performed pixel wise, leading to a failure of the test if the
two images are not identical. In case of non identical pictures, the MSE index of the difference 
image will be calculated and compared to a threshold value that the user can set (by default it 
is 0): if the computed MSE index (whose value will be between 0 and 1) is lower that this 
threshold, then the test will pass, if not it will fail. In both cases the user will be notified 
about the actual value of the MSE index. In the future, a more sophisticated comparison method may
be provided.

- If the dimensions of the test image are different from the ones of the reference picture, then 
the test screenshot will be resized to match the reference one before the comparison is performed. 


## Interface

For RViz itself: the CMake flag `EnableVisualTests` is provided to enable visual tests and the flag
`GenerateReferenceImages` to choose between the two possible behaviours:

- if the option `GenerateReferenceImages` is set to `TRUE`, the reference screenshots will be
taken/updated and no comparison performed:

        ament test --cmake-args -DEnableVisualTests=TRUE -DGenerateReferenceImages=TRUE

- with the option `GenerateReferenceImages` set to `FALSE`, the test screenshots will be taken and
compared to existing reference ones:

        ament test --cmake-args -DEnableVisualTests=TRUE DGenerateReferenceImages=FALSE

As anticipated above, if the tests run (i.e. if `EnableVisualTests` is set to `TRUE` and 
`GenerateReferenceImages` to `FALSE`), by default each test will succeed if both the test images
and the relative reference images exist and they are pixel-wise identical. It will fail if either 
the two sets of images exist but they are not identical, or if at least one of the reference images 
does not exist.

Always by default, the images are named after the test they belong to. In particular the 
reference image
relative to the main render window will be called `<test_name>_ref.png` and the corresponding test 
image will be `<test_name>.png`. For what concerns the secondary windows, they will be named 
similarly: `<test_name>_secondary_window#_ref.png` and `<test_name>_secondary_window#.png`,
where `#` is the number of the secondary window. These names can be changed by providing the 
desired name as an argument to the capture methods (i.e. `captureRenderWindow()`, 
`captureMainWindow()` or `assertMainWindowIdentity()`).

Given that we want image references to be provided to the user in the source directory, it is
important for the tests to be stable when it comes to comparing screenshots taken on different
machines. If the images are taken on different screens, in fact, they will almost certainly not
be pixel-wise identical, so that the pixel-by-pixel comparison will fail even if the test
should pass.  
For the moment this issue is addressed by computing the MSE (Mean Square Error) for the two 
pictures and comparing it to a threshold value that can be set by the user and (by default
equal to 0). NB: the MSE is a value between 0 (if the images are identical) and 1 (if the 
difference image is completely white).  
If the computed MSE is found to be bigger than the set threshold, then the test will fail, 
otherwise it will pass. To set the threshold value, another CMake flag is provided: 
`MSEThreshold`, and the user can set it via command line when starting the tests:

        ament test --cmake-args -DEnableVisualTests=TRUE  -DMSEThreshold=0.001
    
Because of the heuristic nature of this method, the appropriate threshold value will depend on the 
single test and on the context.  

NB: all the three cmake flags values are automatically cached by cmake, meaning that if they are 
not specified, the last set values will be used. 


## GUI interaction

In order to be able to set the scene in the desired way before taking the screenshots (e.g.
add one or more displays, modify their properties, etc.), one would like to be able to
automatise the interaction with the RViz GUI. In the future, on top of being the most sensible and 
effective way to prepare the 3D scene for the visual tests, this functionality may also allow GUI
tests to be performed. This can be achieved with the help of the QTest framework. So 
far the Visual Testing framework offers the possibility to add and interact with most of the 
available displays (Grid, PointCloud, Camera, Image and Polygon), and the addition of more 
displays in the future is made straightforward by the use of a base class from which all
the displays page objects derive:

- The class `BasePageObject` is provided, as the base class for every display page object. It
implements all the basic methods to modify the various kind of display properties (QString, 
boolean, QComboBox, etc.)

- For each display, a class `<DisplayName>DisplayPageObject` - which inherits from 
`BasePageObject` - exists, and it implements the specialised methods to interact with the 
specific properties of the relative displays.

- We also provide a `DisplayHandler` class, which has methods to add displays to the scene and 
to remove them. 

- We provide a test environment, in the form of a TestFixture which offers all the relevant 
functionality for the tests (see relative paragraph below).

## Writing tests

An example of how tests are written and how they work is provided by the `example_test.cpp` file in
rviz_visual_tests/tests. In the following the most important points relative to the tests are 
summarized:

* As said, the `VisualTestFixture` offers convenience methods to:
    * add a new display: `addDisplay<display type>()`, which creates an instance of the desired 
    page object and returns a shared pointer to it.
    * remove a display: `removeDisplay(std::shared_ptr<BasePageObject> display)`.
    * capture a screen shot of the main render window: `captureMainWindow()` or of a secondary 
    window: `captureRenderWindow(std::shared_ptr<PageObjectWithWindow> display)`.
    * assert the identity of test and reference images: `assertScreenShotsIdentity()`.
    * both take a screenshot of the main render window and assert its identity to the relative 
    reference image (which can be used if one is not interested in secondary windows): 
    `assertMainWindowIdentity()`.
    * set the position of the camera and its sight vector: `setCamPose(Ogre::Vector3 pose)` and 
    `setCamLookAt(Ogre::Vector3 look_at)`.

- A custom RViz configuration is loaded right after the application starts. It is the default 
RViz config, with a Grid display and with the help panel hidden.

- In writing the test it is important to take care that the display property that one wants to 
change is visible when the relative display menu is expanded (i.e. there must be enough free 
place in the display panel so that when the display is expanded the desired property row is 
still visible on screen). This is because, in order to set properties, mouse events (in particular
 mouse clicks) are simulated by `QTest`, and if the point we want to click on is not visible on 
screen, then the simulated mouse click will fail (or, rather, the clicked screen area won't 
correspond to the desired one). Note that displays menus are automatically expanded before a 
property is changed. On the other hand, in order to collapse them one can use the 
`BasePageObject` class method `collapse()`.

- For reasons that will become clear later, in each test one and only one of the two methods 
`assertScreenShotsIdentity()` and `assertMainWindowIdentity()` must be called, and after that has
 been done, interaction with RViz is no longer possible for the current test. Any attempt to 
 interact with the GUI after one of those methods has been called will not produce the expected 
 results and will possibly result in a segfault.

- At the end of each test, the scene is cleaned, the application reset and all the present 
displays removed, before the following test begins.

## How tests work

Given their nature, the visual tests need an actual running RViz in order to work.
This means that one needs to have an instance of an `rviz_common::VisualizerApp` and one of an 
associated `QApplication`. The QApplication must then be started and all the interactions with the 
application must be performed while the QApplication is in the main event loop. Unfortunately, 
the method `QApplication::exec()`, called to start RViz, will not return until the QApplication 
is quit, making it impossible to interact with the application in a straightforward way from within 
the same thread. Trying to access the QApplication from a different thread is also not viable, 
due to Qt constraints.

A solution to this problem is provided by the method 
`QTimer::singleShot(int delay, QObject * context, Func1 slot)`, which registers the function `slot`
(either a Q_SLOT or a lambda) to be performed after the amount of time specified by `delay`.
If `QTimer::singleShot()` is called before `QApplication::exec()` and a proper delay is specified, 
then the the action registered by it will take place while the QApplication is still in the main 
event loop, making it possible to interact with RViz.

This mechanism is used in this framework to make the visual tests possible. In particular, the 
class `Executor` takes care of registering with an appropriate delay all the desired actions 
specified in the tests, so that the user does not have to worry about it.

As said, RViz is started by calling `QApplication::exec()`. This is done internally by the 
VisualTestFixture methods which perform the comparison between screenshots (i.e. 
`assertScreenShotsIdentity()` and `assertMainWindowIdentity()`). For this reason, one of these 
methods MUST be called in each test and that must be done only at the end of it. In particular, 
only one of them can be called in one test: `assertScreenShotsIdentity()` is used if one is 
interested also (or only) in secondary render windows and, therefore, screenshots of both the 
main and secondary windows (or only of secondary windows) have been taken, while 
`assertMainWindowIdentity()` can be used if only the main render window is of interest: it will,
in fact, both capture a screenshot of the main render window and perform the image comparison.  
If the method `assertMainWindowIdentity()` is called, there is, therefore, no need to use the 
method `captureMainWindow()` before.

## Test Fixture

As anticipated above, the test environment is provided in the form of a TestFixture, called 
`VisualTestFixture`. Among its members, the fixture owns also an instance of `QApplication`,
`VisualizerApp`, `DisplayHandler` and of `Executor`, so that it can take care of all the tasks 
concerning the startup and shutdown of RViz itself, displays management and GUI interactions 
handling, offering convenience methods for all that can be used in the tests to set the scene, add
or delete a display, capture a screen shot of a render window and compare sets of images.  
