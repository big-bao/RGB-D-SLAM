CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

PROJECT( SLAM )

# 设定可执行二进制的文件目录
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

# 设定存放编译出来的库文件目录
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

# 设定库文件目录为链接目录
LINK_LIBRARIES( ${PROJECT_SOURCE_DIR}/lib ) 

# 设定头文件目录
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )

# 增加子文件夹，进入源代码文件继续构建
#ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src )

add_executable( hello_slam src/hello_slam.cpp)

# 增加 PCL 库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )

# 增加 OpenCV 库的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

add_executable( generate_pointcloud src/generatePointCloud.cpp )
target_link_libraries( generate_pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES} )
