# A quick start guide: how to write and run RViz visual tests
A deeper explanation on what this testing framework is about and on how it works can be found in [this document](documentation.md).

The following serves as a quick introduction to the framework.

## Writing a test

We will look at the following simple example:

```cpp {.line-numbers}
TEST_F(VisualTestFixture, example_test_structure) {
  /// Set the position of the camera and its sight vector (optional):
  setCamPose(Ogre::Vector3(0, 3, 16));
  setCamLookAt(Ogre::Vector3(0, 2, 0));

  /// Add displays:
  auto grid_display = addDisplay<GridDisplayPageObject>();
  auto image_display = addDisplay<ImageDisplayPageObject>();

  /// Modify their properties:
  grid_display->setOffset(0.3, 2, 0.4);
  grid_display->setColor(0, 255, 0);
  grid_display->collapse();
  image_display->setQueueSize(10);

  /// Take the screenshots of the desired render windows:
  captureMainWindow();
  captureRenderWindow(image_display);

  /// Compare test screenshots with the reference ones:
  assertScreenShotsIdentity();
}
```

Let us have a look at what happens:

- The test derives from a test fixture called `VisualTestFixture`.
This fixture provides convenience methods needed for the tests.

- We first change the camera position and its sight vector with the two methods `setCamPose(Ogre::Vector3 pose)` and `setCamLookAt(Ogre::Vector3 look_at)`.
This is only necessary if the default position and sight vector are insufficient.

- To add an RViz display call the method `addDisplay()`, which is templated on the type of the desired display.
In the example above, we first add a Grid display and then an Image display.
The `addDisplay()` method returns a `std::shared_ptr` pointing to the just created display object.
This pointer can be stored locally and used to interact with the display.

- Once a display has been added and a pointer to it stored in a local variable, it can be used to modify the display properties.
Here, we first set the offset of the Grid display to (0.3, 2, 0.4) and then we set its color to green.

- A display can also be collapsed (changing its properties will expand it). This may be useful if there is more than one display present and we want to modify properties of all displays: the property to be modified needs to be visible on screen.
In our example, we collapse the Grid display menu before setting the queue size property of the Image display to a value of 10.

- After the scene is set and all desired displays added and properly adjusted, we can take screenshots of both the main render
window (with the method `captureMainWindow()`) and of secondary render windows belonging to displays (with the method `captureRenderWindow(std::shared_ptr<PageObjectWithRenderWindow> display)`).

- It now remains to compare the screenshots with the reference images. To do so, we call `assertScreenShotsIdentity()`.
It is very important that this method (or the other assert method `assertMainWindowIdentity()`) is called **exactly once** in each test.
After this method has been called in a test, interaction with RViz is no longer possible: every following call to GUI interaction methods will not perform as expected and possibly result in a segfault.

## Running the tests

In order to run the tests, they have to be built first and executed seperately. Use the following commands:

    colcon build --cmake-args -DEnableVisualTests=True -DBUILD_TESTING=1
    
    colcon test

This will make the tests run and the screenshots will be compared to the existing reference images.  
**NB**: CMake will cache the flag value, so that after setting them to `True` or `False`, the following time `colcon test` is run without specifying the flag, the previously set value will be used.

Furthermore, the reference images can be updated by running the tests after setting the environmental variable `GenerateReferenceImages` to `True`.

## Further reading

To have a deeper insight about the possibilities of the framework, please read the [further explanation](documentation.md).
