# 添加C++ 11 标准支持
#cmset(CMAKE_CXX_FLAGS = "-std=c++11")
project(imageBasics)
cmake_minimum_required(VERSION 3.14)

# 寻找OpenCV库
find_package(OpenCV REQUIRED)

find_package(Pangolin)
include_directories( ${Pangolin_INCLUDE_DIRS} )

find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
# 包含头文件目录
include_directories(${PCL_INCLUDE_DIRS})
# Eigen
include_directories("/usr/include/eigen3")

add_executable(stereoVision src/stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(jointMap src/jointMap.cpp)
target_link_libraries(jointMap ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES} Sophus::Sophus)