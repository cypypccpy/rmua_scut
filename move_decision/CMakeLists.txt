project(move_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  roborts_costmap
  )

find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
  )

catkin_package()

add_executable(behavior_test
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/frozen_behavior.h  
  example_behavior/search_behavior.h
  example_behavior/bullet_behavior.h
  example_behavior/blood_behavior.h
  example_behavior/direction_behavior.h
  example_behavior/goal_behavior.h
  example_behavior/escape_behavior.h
  behavior_test.cpp
  executor/chassis_executor.cpp
  executor/gimbal_executor.cpp
  )

target_link_libraries(behavior_test
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(behavior_test
  roborts_msgs_generate_messages)
