cmake_minimum_required(VERSION 3.0.2)
project(mvros)

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  camera_info_manager
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mvros
  CATKIN_DEPENDS roscpp visualization_msgs cv_bridge image_transport sensor_msgs std_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OpenCV_INCLUDE_DIRS})
add_executable(mvros /mvros/src/main.cpp)
TARGET_LINK_LIBRARIES(mvros libopencv_calib3d.so libopencv_core.so
libopencv_features2d.so libopencv_flann.so libopencv_highgui.so libopencv_imgproc.so libopencv_ml.so libopencv_objdetect.so libopencv_photo.so libopencv_video.so libopencv_videostab.so libMVSDK.so)
target_link_libraries(mvros ${catkin_LIBRARIES})
target_link_libraries(mvros ${OpenCV_LIBS})
target_link_libraries(mvros ${Boost_LIBRARIES})

