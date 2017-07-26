cmake_minimum_required(VERSION 2.8.3)
project(fantastic_marker_detection)

## compile in c++ 11
## necessary for a few std::string functions in fmd_consolidation.cpp
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    image_transport
    roscpp
    rospy
    std_msgs
    std_srvs
    sensor_msgs
    camera_info_manager
    cv_bridge
    message_generation
    geometry_msgs
    tf
    tf2
    tf2_eigen
    tf2_kdl
    tf2_ros
    tf2_geometry_msgs
    tf2_msgs
)


set(OpenCV_DIR "/home/kastan/workspace/ThirdParty/opencv/build" )
find_package( OpenCV REQUIRED)

add_message_files(
    FILES
    Marker.msg
    MarkerArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
     message_runtime
     geometry_msgs
     std_msgs
  #LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(  include
                      ${OpenCV_INCLUDE_DIRS}
                      ${catkin_INCLUDE_DIRS}
                      ${Vimba_INCLUDE_DIRS}
                      ~/workspace/ThirdParty/Vimba_2_1/VimbaCPP/Include/  # DARPA's gigE cameras
                      ~/workspace/ThirdParty/Vimba_2_1/VimbaImageTransform/Include/  # transform to OpenCV
                      ~/workspace/ThirdParty/Vimba_2_1/                   # helps Vimba's .h files call more includes
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}     src/main.cpp)
add_dependencies(${PROJECT_NAME}   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# NOTE: I just added this "add_dependencies" thing.  Maybe necessary when using custom messages.

target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(fmd_consolidation     src/fmd_consolidation.cpp)
add_dependencies(fmd_consolidation   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( fmd_consolidation  ${catkin_LIBRARIES})


# set the path to the library folder
link_directories(/home/kastan/workspace/ThirdParty/Vimba_2_1/VimbaCPP/DynamicLib/x86_64bit)

add_executable(vimba     src/vimba.cpp)

target_link_libraries( vimba
    ${catkin_LIBRARIES}
    VimbaCPP
    )

# link the libraries to the executable
# target_link_libraries (my_node foo)


install(FILES
    fantastic_marker_detection.launch
    # myfiles2
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

