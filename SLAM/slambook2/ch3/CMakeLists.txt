cmake_minimum_required(VERSION 2.8)
project(chapter3)

include_directories("/usr/include/eigen3")
set(Eigen3_DIR CMAKE_INSTALL_PREFIX/share/eigen3/cmake)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 加入各子工程
add_subdirectory(useEigen)
add_subdirectory(useGeometry)
add_subdirectory(visualizeGeometry)
add_subdirectory(examples)
