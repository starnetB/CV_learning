cmake_minimum_required(VERSION 3.14)
project(eigenMatrix)
set(CMAKE_CXX_STANDARD 17)

# 设置输出根目录为build/Debug
set(OUTPUT_DIRECTORY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE})
# 设置可执行程序输出到build/Debug/bin目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
# 设置库文件输出到build/Debug/lib目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Archive directory" FORCE)

set(PANGO_LIB_DIR /Users/lidanqing/Work/CV_learning/Pangolin/devel/lib)


find_library(pango_lib pangolin
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango: ${pango_lib}")

include_directories("/usr/include/eigen3")
set(Eigen3_DIR CMAKE_INSTALL_PREFIX/share/eigen3/cmake)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories("/Users/lidanqing/Work/CV_learning/Pangolin/devel/include")
include_directories("/usr/local/include")
find_package(OpenGL REQUIRED)
find_package(Sophus REQUIRED)

add_executable(useSophus src/useSophus.cpp)
target_link_libraries(useSophus Eigen3::Eigen Sophus::Sophus)

add_executable(trajectoryError src/trajectoryError.cpp)
target_link_libraries(trajectoryError Eigen3::Eigen Sophus::Sophus ${pango_lib})


