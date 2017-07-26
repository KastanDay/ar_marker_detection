# Fantastic Marker Detection

AR marker detection for Autonomous In-Space Assembly project. Using OpenCV and OpenCV_Contrib --> ArUco module for detection.

Currently set to work with ROS's usb_cam for testing with USB webcams (http://wiki.ros.org/usb_cam) 

### Dependencies

1. ROS
2. Optional cam package: [ROS USB\_cam](www.wiki.ros.org/usb_cam)
3. Optional cam package: [Vimba SDK](https://www.alliedvision.com/en/products/software.html) - for Allied Vision (DARPA) cams


### Building
Use `$ catkin_make` or `$ catkin_make fantastic_marker_detection` in the root `catkin_ws/` directory to build.

## Running
Process:
Launch camera publisher(s), then launch `fantastic_marker_detection` and (potentially) change the `.launch` file to subscribe to your camera feed.

1. Edit the `fantastic_marker_detection.launch` file to:
  1. Number of `fantastic_marker_detection` nodes should be equal to your number of camera feeds.
  2. Ensure each node subscribes to the proper ros\_topic feed.
  3. Ensure the `static_transform_publisher` nodes have the proper transform parameters (input the position and pose the various cameras.
  4. Finally, ensure that source and target frames for each node (camera fed) match the names of the source and target names of the `static_transform_publisher`

1. `$ roslaunch fantastic_marker_detection gigE_cams.launch`

  * To run the Allied Vision GigE cameras (from DARPA), run node gigE.
This will start publishing the info from the DARPA cams to ros-topics.

2. `$ roslaunch fantastic_marker_detection fantastic_marker_detection.launch`
  * This will launch an individual node for each camera feed to which you would like to subscribe and process.
If you want to run this program on more cameras, or on different feeds, *edit the launch file*


- You can also run any other camera publisher you desire!  Just change `fantastic_marker_detection.launch` to subscribe to your ros\_topic feed!

## All params that could require attention:
1. Topics to which you must subscribe
2. Frames from which you must transform.
3. 

# Program design:

All marker logic is contained in `marker.cpp` and `fmd_consolidation.cpp`. All other programs are just various ways of publishing camera data on ros\_topics.



### marker_detection.cpp
- Purpose:
  1. 
  2. Transformation. 
- Structure: 
  - One node per one camera feed.  Launch multiple nodes to handle multiple camera feeds from the `fantastic_makrer_detection.launch` file.

### fmd_consolidation.cpp
- Purpose:
  1. Take in all 

### vimba.cpp
- Purpose:
  1. Detect all Allied Vision gigE cameras connected to the local netowrk.
  2. Take in, convert, then publish that camera data to individual ros\_topics. 
    - It publishes sensor_msgs::ImagePtr messages
    - Topics by default: "gigE\_cam\_0", "gigE\_cam\_1", etc.


### Creating Markers 

_There's no NEED to create new markers.  But if you want your own set (or a larger set) you can with this._

Note: There is no build file (make or cmake) for this program included in this repo right now. Compile with g++.

Dictionaries of markers can be created using the program 'automaticDictGen.cpp'

Example: 

`$./automaticDictGen --bb=1 -ms=500 -numMark=64 -markDim=4 "/home/kastan/aruco/markers/64_4x4" # note NO trailing '/' #`

```
    -bb = marker border (reccomended = 1, need some border to do identification)
    -ms = size of marker in pixels.  500 = 500x500 total
    -numMark = number of markers in dictionary. Smaller is better to make identification faster and better.
    -markDim = dimension of marker aka how many squares it is made of (4 = 4x4 markers, 5 = 5x5 markers)
    "output/dir" = location where marker images (png) will be written.  MUST include the trailing '/'
```
I will also say: 4x4 markers seem to be the best balance between reducing false positives and keeping the markers small.

3x3 = *lots* of false positives (esp. with computer keyboards).

8x8 = marker has to be physically larger to be identified.


### Contributors
Written by Kastan Day during the summer of 2017.
I just finished my first year at Swarthmore College before starting this internship. I would very happy to help as much as I can in the future, just email me here: kastanvday[at]gmail[dot]com
