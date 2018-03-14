# RViz 3D Visual Testing Framework

This is a testing framework which allows the user to take screenshots of the 3D rendered scene and
compare them with previously captured reference screenshots.


## Assumptions and Settings

- Visual tests are located in the folder rviz_visual_tests.

- The reference screenshots are located in the rviz_visual_tests/tests source folder, in a
directory called reference_images. The test images are, instead, located in the rviz_visual_tests
build folder, in a directory called test_images, generated automatically at build system generation.

- Visual tests are especially useful for regression testing.

- The user will be able to do both of the following: either take/update reference images, or take
test images and compare them with existing references.


## Interface

For RViz itself: the CMake flag EnableVisualTests is provided to enable visual tests and the flag
GenerateReferenceImages to choose between the two possible behaviours:

- if the option GenerateReferenceImages is set to TRUE, the reference screenshots will be
taken/updated and no comparison performed:

        ament test --cmake-args -DEnableVisualTests=TRUE -DGenerateReferenceImages=TRUE

- with the option GenerateReferenceImages set to FALSE, the test screenshots will be taken and
compared to existing reference ones:

        ament test --cmake-args -DEnableVisualTests=TRUE DGenerateReferenceImages=FALSE

If the tests run (i.e. if EnableVisualTests is set to TRUE and GenerateReferenceImages to FALSE),
each test will succeed if both the test image and the relative reference image exist and they are
identical. It will fail if either the two images exist but they are not identical, or if the
reference image does not exist.

By default, the images are named after the test they belong to, in particular the reference image
will be called <test_name>_ref.png and the test image will be <test_name>.png. This can be changed
by providing an argument to the method assertImageIdentity() (this should be done in case one wants
to take more than one screenshot in the same test, otherwise only the last image will be saved)

Given that we want image references to be provided to the user in the source directory, it is
important for the tests to be stable when it comes to comparing screenshots taken on different
machines. If the images are taken on different screens, in fact, they will almost certainly not
be pixelwise identical, so that the pixel-by-pixel comparison will fail even if the test
should pass. For the moment this issue is addressed by computing the difference image and then
the average pixel intensity of this one (see last point in the paragraph
'Tested image comparison tools' below). If this value  is found to be under a given threshold,
then the test will pass, if not it will fail. To set the said threshold value, another
CMake flag is provided: ComparisonThreshold, and the user can set it via command line when
starting the tests:

    ament test --cmake-args -DEnableVisualTests=TEST  -DComparisonThreshold=0.0001
    // this will make the tests run with a threshold value of 0.0001.


## GUI interaction

In order to be able to set the scene in the desired way before taking the screenshots (e.g.
add one or more displays, modify their properties, and so on), one would like to be able to
automatise the interaction with the RViz GUI. On top of being the most sensible and effective
way to prepare the 3D scene for the visual tests, this functionality would also allow GUI tests
to be performed, i.e. it would make it possible, for example, to test if the right widgets are
displayed, if they show the right text and if they behave as one would expect. All this can be
achieved with the help of the QTest framework. So far the Visual Testing framework offers the
possibility to add and interact with PointCloud and Grid displays, and the addition of future
displays in the future is made straightforward by the use of a base class from which all the
displays page objects derive:

- The class BasePageObject is provided, as the base class for every display page object. It
implements methods to add a display, to remove it or all the present displays, and all the
methods to modify the various kind of the display properties (QString, boolean, QComboBox, etc.)

- For each display a class <DisplayName>DisplayPageObject - which inherits from BasePageObject -
exists, and it implements the specialised methods to interact with the specific property of
the relative displays.

- We provide a test environment, in the form of a TestFixture which, among the others,
has a method to add new displays. This addDisplay() method is templated on the display kind.


## Writing tests

- As said, a test environment is provided, in the form of a TestFixture.

- We provide a function that either takes/updates reference images or takes test screenshots, and,
in this last case, proceeds to perform the comparison.

- Functions to set the position and direction of the camera are provided.

- The scene is cleaned and reset before a new test begins, and all the present displays are removed.

An example of how a test is written and how it works is provided by the example_test.cpp file in
rviz_visual_tests/tests.
