# 查找当前目录下的所有源文件

# 并将名称保存到 DIR_LIB_SRCS 变量

aux_source_directory(. DIR_SOCKETCOMM)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})


# 生成链接库

add_library (Socket ${DIR_SOCKETCOMM})

target_include_directories(Socket PUBLIC include)

#target_include_directories(GyroScope PUBLIC ../../SerialPort/include)

