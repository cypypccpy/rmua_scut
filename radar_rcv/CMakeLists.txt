# CMake 最低版本号要求

#cmake_minimum_required (VERSION 2.8)
cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# set(CMAKE_BUILD_TYPE Debug)

# 项目信息

project (radar_rcv LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)
add_message_files(DIRECTORY msg FILES CarCoordinate.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(Socket)
include_directories(${catkin_INCLUDE_DIRS})

# 查找当前目录下的所有源文件

# 并将名称保存到 DIR_SRCS 变量

aux_source_directory(. DIR_SRCS)

# 添加 math 子目录


add_subdirectory(Socket)

add_executable(sub sub.cpp)
target_link_libraries(sub ${catkin_LIBRARIES})
target_link_libraries(sub ${OpenCV_LIBS})
target_link_libraries(sub Socket)

