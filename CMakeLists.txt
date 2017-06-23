cmake_minimum_required(VERSION 2.8.3)
project(fantasticMarkerDetection)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS image_transport roscpp std_msgs std_srvs sensor_msgs camera_info_manager cv_bridge)

set(OpenCV_DIR "/home/kastan/workspace/ThirdParty/opencv/build" )
find_package( OpenCV REQUIRED)

message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
message("OpenCV includes:  "  ${OpenCV_INCLUDE_DIRS} )
message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(  include
                      ${OpenCV_INCLUDE_DIRS}
                      ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}   src/main.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}

)

