# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)

# 项目信息
project (rosbag_parse)

SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE "Debug")
#MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -g  -march=native  ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g  -march=native ")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

ADD_DEFINITIONS(-O3 -g -W -Wall)
#ADD_DEFINITIONS(-O0 -g -W -Wall)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    tf
    std_msgs
    roscpp
    pcl_ros
    sensor_msgs
    sirius_msgs
)

# OpenCV
find_package( OpenCV REQUIRED )
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

catkin_package(CATKIN_DEPENDS
    tf
    roscpp
    sensor_msgs
    std_msgs
    sirius_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# 查找目录下的所有源文件，并保存至DIR_SRCS
aux_source_directory(src DIR_SRCS)

# 指定生成目标
add_executable(rosbag_parse_node ${DIR_SRCS})

# 添加链接库
target_link_libraries(rosbag_parse_node
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
     stdc++)
