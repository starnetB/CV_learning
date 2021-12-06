cmake_minimum_required(VERSION 3.14)
project(PCLDemo)
#set(CMAKE_CXX_STANDARD 14)

# 设置输出根目录为build/Debug
set(OUTPUT_DIRECTORY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE})
# 设置可执行程序输出到build/Debug/bin目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
# 设置库文件输出到build/Debug/lib目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Archive directory" FORCE)

find_package(PCL REQUIRED)
# 包含头文件目录
include_directories(${PCL_INCLUDE_DIRS})
# 设置依赖库链接目录
link_directories(${PCL_LIBRARY_DIRS})
# 添加预处理器和编译器标记
add_definitions(${PCL_DEFINITIONS})

add_executable(PCLDemo src/main.cpp)
target_link_libraries(PCLDemo ${PCL_LIBRARIES})

add_executable(CloudDemo src/cloud_viewer.cpp)
target_link_libraries(CloudDemo ${PCL_LIBRARIES})

add_executable(pcl_visualizer src/pcl_visualizer.cpp)
target_link_libraries(pcl_visualizer ${PCL_LIBRARIES})

add_executable(pcl_transform src/pcl_transform.cpp)
target_link_libraries(pcl_transform ${PCL_LIBRARIES})

add_executable(load_pcd src/load_pcd.cpp)
target_link_libraries(load_pcd ${PCL_LIBRARIES})

add_executable(save_pcd src/save_pcd.cpp)
target_link_libraries(save_pcd ${PCL_LIBRARIES})

add_executable(kd_tree src/kd_tree.cpp)
target_link_libraries(kd_tree ${PCL_LIBRARIES})

add_executable(octree src/octree.cpp)
target_link_libraries(octree ${PCL_LIBRARIES})



# 4.application 点云的使用
add_executable(moment_of_inertia src/4_application/moment_of_inertia.cpp)
target_link_libraries(moment_of_inertia ${PCL_LIBRARIES})

# 2 点云进阶
add_executable(PassThrough src/2_ad/PassThrough.cpp)
target_link_libraries(PassThrough ${PCL_LIBRARIES})

add_executable(downsample_voxel_grid src/2_ad/downsample_voxel_grid.cpp)
target_link_libraries(downsample_voxel_grid ${PCL_LIBRARIES})
