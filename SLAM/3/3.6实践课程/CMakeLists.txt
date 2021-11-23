cmake_minimum_required(VERSION 3.14)
project(eigenMatrix)
#set(CMAKE_CXX_STANDARD 14)

# 设置输出根目录为build/Debug
set(OUTPUT_DIRECTORY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE})
# 设置可执行程序输出到build/Debug/bin目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
# 设置库文件输出到build/Debug/lib目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Archive directory" FORCE)

include_directories("/usr/include/eigen3")
set(Eigen3_DIR CMAKE_INSTALL_PREFIX/share/eigen3/cmake)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


add_executable(useGeometry src/useGeometry.cpp)
target_link_libraries(useGeometry Eigen3::Eigen)


add_executable(coordinateTransfrom src/coordinateTransfrom.cpp)
target_link_libraries(coordinateTransfrom Eigen3::Eigen)