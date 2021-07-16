cmake_minimum_required(VERSION 2.8.3)
project(vid_orbslam3)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_CXX_STANDARD 14)
set(ORBSLAM3_LIB_NAME libORB_SLAM3)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf  
  image_transport
  cv_bridge
  message_generation
  geometry_msgs
  message_filters
  #dji_osdk_ros
)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   KeyframeGraph.msg
   KeyframeStamped.msg
   TrajectoryPoints.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vid_orbslam3
  CATKIN_DEPENDS roscpp std_msgs tf geometry_msgs message_filters #dji_osdk_ros 
  DEPENDS system_lib message_runtime
)

###########
## Build ##
###########

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(COLMAP REQUIRED)
find_package (GeographicLib REQUIRED)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ~/catkin_ws/src/vid_orbslam3/include
  ~/catkin_ws/src/ORB_SLAM3/include
  ~/catkin_ws/src/ORB_SLAM3/include/CameraModels
  #~/catkin_ws/src/OnBoard-SDK-ROS/include/dji_osdk_ros
  #~/Onboard-SDK/build/install/include
  ${COLMAP_INCLUDE_DIRS}
)
link_directories(~/catkin_ws/src/ORB_SLAM3/lib
~/Onboard-SDK/build/install/lib
#~/catkin_ws/src/ORB_SLAM3/Thirdparty/DBoW2/lib
#~/catkin_ws/src/ORB_SLAM3/Thirdparty/g2o/lib
${COLMAP_LINK_DIRS}
)
add_executable(vid_orbslam3 src/SubscribeAndPublish.cpp src/ImuGrabber.cpp)
#add_executable(calc_transformation_bc src/calc_transformation_bc.cpp src/ImuGrabber.cpp)
#add_executable(control_gimbal src/control_gimbal.cpp)
add_executable(gps_process src/GPSProcess.cpp)
add_executable(static_transform src/StaticTransformCalculation.cpp)

add_dependencies(vid_orbslam3 ${catkin_EXPORTED_TARGETS})
#add_dependencies(calc_transformation_bc ${catkin_EXPORTED_TARGETS})
#add_dependencies(control_gimbal ${catkin_EXPORTED_TARGETS})
add_dependencies(gps_process ${catkin_EXPORTED_TARGETS})
add_dependencies(static_transform ${catkin_EXPORTED_TARGETS})

add_library(${ORBSLAM3_LIB_NAME} SHARED IMPORTED)
set_target_properties(${ORBSLAM3_LIB_NAME} PROPERTIES IMPORTED_LOCATION "~/catkin_ws/src/ORB_SLAM3/lib/libORB_SLAM3.so")


## Specify libraries to link a library or executable target against
target_link_libraries(vid_orbslam3
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
   ${Pangolin_LIBRARIES}
   ORB_SLAM3
   #dji_osdk_ros
   ~/catkin_ws/src/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
   ~/catkin_ws/src/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
-lboost_serialization
 )
target_link_libraries(gps_process
   ${catkin_LIBRARIES}
   ${COLMAP_LIBRARIES}
   ${GeographicLib_LIBRARIES}
   -lboost_serialization
)
target_link_libraries(static_transform
   ${COLMAP_LIBRARIES}
   ${catkin_LIBRARIES}
-lboost_serialization
)


