# Display types

Basic documentation about display types can be found here: http://wiki.ros.org/rviz/DisplayTypes
Here, we only add documentation about additional or changed features:

## Camera Display

As before, the camera display needs a `sensor_msgs:msg::Image` topic to subscribe to.
In addition and as before, it needs information about the camera via a `sensor_msgs::msg::CameraInfo` message. 

*Changed behaviour:* The CameraInfo topic is assumed to be relative to the Image topic: 
If the image topic is "/image", the CameraInfo topic is assumed to be "/image/camera_info".

## Map Display

#### Topics
* Maps process data from `nav_msgs::msg::OccupancyGrid` messages and can update these maps also through `map_msgs::msg::OccupancyUpdate`. 
* If the map topic is "/map_topic", the display assumes update messages to be published on the topic "/map_topic_update"

#### Additional properties
* *Draw behind:* Always draw the map in the background of the whole scene (ignore perspective)
* *Use timestamp:* Use the timestamp of the map when transforming into local frames
* *Color scheme:* Maps can be displayed with different color schemes

#### Possible color schemes

The OccupancyGrid message contains maps with values as uint8, i.e. 256 signed possible values. 
These are mapped to color schemes. 
In general, values between 0 and 100 are regarded as valid (often interpreted as occupancy probabilities).

* *Raw:* Paint all points from black (0) to white (255).
* *Map:* Paint valid points between 0 and 100 from white (0) to black (100). 
  Paint valid value -1 in a blueish, greenish, grayish color (it might be used to indicate unchartered territory for instance). 
  Invalid points between 101 and 127 are painted in green, while invalid negative numbers are painted in shades from red to yellow.
* *Costmap:* Paint valid points between 0 and 98 from blue to red. 
  Paint points with value 99 in cyan (obstacle value) and points with value 100 in purple (lethal obstacle). 
  The valid value -1 is painted in a blueish, greenish, grayish color. 
  Invalid points between 101 and 127 are painted in green, while invalid negative numbers are painted in shades from red to yellow.

## Odometry Display

The odometry display can include and display covariance properties, using the same mechanism as the PoseWithCovarianceStamped.
To do so, the odometry message must contain a covariance matrix (6 x 6 matrix with covariances, may contain only 2d information by leaving the 3rd, 4th and 5th row and column empty).

