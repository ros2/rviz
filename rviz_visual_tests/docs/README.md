# A quick start guide: how to write and run RViz visual tests
A deeper explanination on what this testing framework is about and on how it works can be found 
in [this document](in_depth_explaination.md).  
The following serves as a quick introduction to the framework.
 
## Writing a test

To explain how a visual test can be written, let's take a look at a simple example:

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
      image_display->setQueueSize("10");
        
      /// Take the screenshots of the desired render windows:
      captureMainWindow();
      captureRenderWindow(image_display);
        
      /// Compare test screenshots with the reference ones:
      assertScreenShotsIdentity();
    } 
```

Examining this example we can learn the following:

- The tests derive from a test fixture (called `VisualTestFixture`). This fixture provides all 
the convenience methods needed for the tests.

- If the default settings of the camera are not the desired ones, one can change both its position 
and its sight vector with the two methods `setCamPose(Ogre::Vector3 pose)` and
`setCamLookAt(Ogre::Vector3 look_at)`.

- To add an RViz display, the method `addDisplay()` is used, templated on the type of the 
desired display. In the example above, we first add a Grid display and then an Image display. 
The `addDisplay()` method returns a `std::shared_ptr` pointing to the just created display object. 
This pointer can be stored locally and used to interact with the display.

- Once a display has been added and a pointer to it stored in a local variable, this one can be 
used to modify the display properties.  
In this example, we first set the offset of the Grid display to (0.3, 2, 0.4) and then we set its
color to green. A display can also be collapsed (changing its properties will expand it). This 
may be useful if there is more than one display present and we want to modify properties of all
displays: the property to be modified, in fact, need to be visible on screen.  
In our example, we collapse the Grid display menu before setting the queue size property of the 
Image display to a value of 10.

- After the scene is set and all the desired displays added and properly adjusted, we can take 
screenshots of both the main render window (with the method `captureMainWindow()`) and of 
secondary render windows belonging to the displays (with the method 
`captureRenderWindow(std::shared_ptr<PageObjectWithRenderWindow> display)`).

- At this point, we have the screenshots that we wanted and all it's left to do is to compare 
them with the reference images. To do so, we call the method `assertScreenShotsIdentity()`.  
It is very important that this is the last thing done in the test, and that it is done. In other 
words, this method MUST be called in every test, ant that must be done at the very end of it.

## Running the tests

In order to run the tests, we use the following command:
    
    ament test --cmake-args -DEnableVisualTests=TRUE

This will make the tests run and the screenshots will be compared to the existing reference 
images. If, instead, one wants to update the referencce pictures, the following command must be 
used:

    ament test --cmake-args -DEnableVisualTests=TRUE -DGenerateReferenceImages=TRUE
    
N.B. cmake will cache the flags values, so that after setting them to `TRUE` or `FALSE`,  the 
following time `ament test` is run without specifying any cmake flag, the previoulsy set values 
will be used.

## Further reading

As said above, this is only a quick start guide on how to write an RViz visual test. In order to 
have a deeper insight about the framework and its functionality, we suggest to read also this 
[further explaination](in_depth_explaination.md).
