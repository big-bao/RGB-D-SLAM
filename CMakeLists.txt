CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

PROJECT( SLAM )

#设定可执行二进制的文件目录
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

#设定存放编译出来的库文件目录
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

#设定库文件目录为链接目录
LINK_LIBRARIES( ${PROJECT_SOURCE_DIR}/lib ) 

#设定头文件目录
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )

#增加子文件夹，进入源代码文件继续构建
#ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src )

add_executable( hello_slam src/hello_slam.cpp)

