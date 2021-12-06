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

set(PANGO_LIB_DIR /home/franka/home/CV_learning/Pangolin/devel/lib)

find_library(pango_core_lib pango_core
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_core: ${pango_core_lib}")

find_library(pango_display_lib pango_display
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_display: ${pango_display_lib}")

find_library(pango_geometry_lib pango_geometry
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_geometry: ${pango_geometry_lib}")

find_library(pango_glgeometry_lib pango_glgeometry
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_glgeometry: ${pango_glgeometry_lib}")

find_library(pango_image_lib pango_image
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_image: ${pango_image_lib}")

find_library(pango_opengl_lib pango_opengl
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_opengl: ${pango_opengl_lib}")

find_library(pango_packetstream_lib pango_packetstream
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_packetstream: ${pango_packetstream_lib}")

find_library(pango_plot_lib pango_plot
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_plot: ${pango_plot_lib}")

find_library(pango_python_lib pango_python
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_python: ${pango_python_lib}")

find_library(pango_scene_lib pango_scene
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_scene: ${pango_scene_lib}")

find_library(pango_tools_lib pango_tools
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_tools: ${pango_tools_lib}")

find_library(pango_vars_lib pango_vars
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_vars: ${pango_vars_lib}")

find_library(pango_video_lib pango_video
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_video: ${pango_video_lib}")

find_library(pango_windowing_lib pango_windowing
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib pango_windowing: ${pango_windowing_lib}")

find_library(tinyobj_lib tinyobj
        HINTS ${PANGO_LIB_DIR}
        REQUIRED)
MESSAGE(STATUS "lib tinyobj: ${tinyobj_lib}")

include_directories("/usr/include/eigen3")
set(Eigen3_DIR CMAKE_INSTALL_PREFIX/share/eigen3/cmake)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories("/home/franka/home/CV_learning/Pangolin/devel/include")

find_package(OpenGL REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})


add_executable(useSophus src/useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus Eigen3::Eigen GL GLEW)