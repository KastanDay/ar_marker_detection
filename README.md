Welcome!

AR marker detection for Autonomous In-Space Assembly project. Using OpenCV and OpenCV_Contrib --> ArUco module for detection.

Currently set to work with ROS's usb_cam for testing with USB webcams (http://wiki.ros.org/usb_cam) 

### Dependencies

1. ROS

2. USB_Cam
http://wiki.ros.org/usb_cam


### Building
Use `$ catkin_make` in the root `catkin_ws/` directory to build.

### Detecting Markers 

First: launch camera publisher which our detector will subscribe to:

`$ roslaunch  usb_cam  usb_cam-test.launch`

Then launch our detector
`$ ./fantasticMarkerDetection # no parameters necessary for good detection`

### Creating Markers 

Note: There is no build file (cmake or make) for this program included in this repo right now.

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

3x3 = *lots* of false positives.

8x8 = marker has to be larger to be identified.

