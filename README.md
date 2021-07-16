# vid_orbslam3 package
The vid_orbslam3 package provides Keyframe graph publishing using ORBSLAM3 and geo-referencing fuctionality.

## Pre-requirements
* ORBSLAM3
* ROS
* OpenCV
* colmap
* GeographicLib

## Installation
* Install ROS, GeographicLib and colmap using their official documentation.
* Follow the instruction of ORBSLAM3 documentation and compile it inside the ROS workspace Ex: catkin_ws/src
* To extend the fuctionality to access the Atlas and keyframes from ORBSLAM3, small addition has to be done manually. Add the following lines into System.h and System.cc respectively.

Inside include/System.h
```
Atlas* getAtlas();
```
Inside src/System.cc
```
Atlas* System::getAtlas(){

	return mpAtlas;
}
```
* Recompile the ORBSLAM3.
* Add Thirdparty folder available in ORBSLAM3 folder into the include folder. (This package requirement as developed)
* Clone this repo, into ROS workspace.
* Compile the package with the following commands.
 ```
$ cd catkin_ws/
$ catkin_make --pkg vid_orbslam3
```
## Camera calibration
ORBSLAM3 requires camera intrinsic parameters as input in it's settings file. Use standard ROS camera calibrator to calibrate the camera. Update the yaml file inside the setting folder as required.

## Usage
1. Run roscore.
2. Make sure the camera topic /dji_osdk_ros/main_camera_images or any other camera topic( Change topic name in main() method if required) and /dji_osdk_ros/gps_position (change gps topic name in gps_process.launch if required) is running.
3. Use the following commands to run the vid_orbslam3 node and geo-referencing node.
```
$ roslaunch vid_orbslam3 vid_orbslam3.launch
$ roslaunch vid_orbslam3 gps_process.launch
```
### Output
A topic named /GPSProcess/KeyframeGraphInENU is published as ros topic. This is basically a posearray which is updated continously as and when the gps_process node gets a new keyframegraph msg from vid_orbslam3 node.
