# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ziye01/CV_learning/ceres-solver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ziye01/CV_learning/ceres-solver/build

# Include any dependencies generated for this target.
include internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/depend.make

# Include the progress variables for this target.
include internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/progress.make

# Include the compile flags for this target's objects.
include internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/flags.make

internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/cuda_sparse_matrix_test.cc.o: internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/flags.make
internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/cuda_sparse_matrix_test.cc.o: ../internal/ceres/cuda_sparse_matrix_test.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ziye01/CV_learning/ceres-solver/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/cuda_sparse_matrix_test.cc.o"
	cd /home/ziye01/CV_learning/ceres-solver/build/internal/ceres && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cuda_sparse_matrix_test.dir/cuda_sparse_matrix_test.cc.o -c /home/ziye01/CV_learning/ceres-solver/internal/ceres/cuda_sparse_matrix_test.cc

internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/cuda_sparse_matrix_test.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cuda_sparse_matrix_test.dir/cuda_sparse_matrix_test.cc.i"
	cd /home/ziye01/CV_learning/ceres-solver/build/internal/ceres && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ziye01/CV_learning/ceres-solver/internal/ceres/cuda_sparse_matrix_test.cc > CMakeFiles/cuda_sparse_matrix_test.dir/cuda_sparse_matrix_test.cc.i

internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/cuda_sparse_matrix_test.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cuda_sparse_matrix_test.dir/cuda_sparse_matrix_test.cc.s"
	cd /home/ziye01/CV_learning/ceres-solver/build/internal/ceres && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ziye01/CV_learning/ceres-solver/internal/ceres/cuda_sparse_matrix_test.cc -o CMakeFiles/cuda_sparse_matrix_test.dir/cuda_sparse_matrix_test.cc.s

# Object files for target cuda_sparse_matrix_test
cuda_sparse_matrix_test_OBJECTS = \
"CMakeFiles/cuda_sparse_matrix_test.dir/cuda_sparse_matrix_test.cc.o"

# External object files for target cuda_sparse_matrix_test
cuda_sparse_matrix_test_EXTERNAL_OBJECTS =

bin/cuda_sparse_matrix_test: internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/cuda_sparse_matrix_test.cc.o
bin/cuda_sparse_matrix_test: internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/build.make
bin/cuda_sparse_matrix_test: lib/libgtest.a
bin/cuda_sparse_matrix_test: lib/libtest_util.a
bin/cuda_sparse_matrix_test: lib/libceres.a
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/libspqr.so
bin/cuda_sparse_matrix_test: lib/libgtest.a
bin/cuda_sparse_matrix_test: lib/libceres.a
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/libspqr.so
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/libtbb.so
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/libglog.so
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/libcholmod.so
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/libamd.so
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/libcamd.so
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/libccolamd.so
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/libcolamd.so
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
bin/cuda_sparse_matrix_test: /usr/local/cuda-11.0/lib64/libcudart_static.a
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/librt.so
bin/cuda_sparse_matrix_test: /usr/local/cuda-11.0/lib64/libcublas.so
bin/cuda_sparse_matrix_test: /usr/local/cuda-11.0/lib64/libcusolver.so
bin/cuda_sparse_matrix_test: /usr/local/cuda-11.0/lib64/libcusparse.so
bin/cuda_sparse_matrix_test: lib/libceres_cuda_kernels.a
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/liblapack.so
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/libblas.so
bin/cuda_sparse_matrix_test: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
bin/cuda_sparse_matrix_test: internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ziye01/CV_learning/ceres-solver/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/cuda_sparse_matrix_test"
	cd /home/ziye01/CV_learning/ceres-solver/build/internal/ceres && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cuda_sparse_matrix_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/build: bin/cuda_sparse_matrix_test

.PHONY : internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/build

internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/clean:
	cd /home/ziye01/CV_learning/ceres-solver/build/internal/ceres && $(CMAKE_COMMAND) -P CMakeFiles/cuda_sparse_matrix_test.dir/cmake_clean.cmake
.PHONY : internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/clean

internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/depend:
	cd /home/ziye01/CV_learning/ceres-solver/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ziye01/CV_learning/ceres-solver /home/ziye01/CV_learning/ceres-solver/internal/ceres /home/ziye01/CV_learning/ceres-solver/build /home/ziye01/CV_learning/ceres-solver/build/internal/ceres /home/ziye01/CV_learning/ceres-solver/build/internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : internal/ceres/CMakeFiles/cuda_sparse_matrix_test.dir/depend

