# CMakeLists.txt to build hellocuda.cu
cmake_minimum_required(VERSION 2.8)
find_package(CUDA QUIET REQUIRED)
 
# Specify binary name and source file to build it from
#add_library(utils utils.cpp)
cuda_add_executable(
    squaresum
    test.cpp squaresum.cu utils.cu)
#target_link_libraries(squaresum utils)