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
include internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/depend.make

# Include the progress variables for this target.
include internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/progress.make

# Include the compile flags for this target's objects.
include internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/flags.make

internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/ba_iterschur_suitesparse_clustjacobi_auto_test.cc.o: internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/flags.make
internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/ba_iterschur_suitesparse_clustjacobi_auto_test.cc.o: ../internal/ceres/generated_bundle_adjustment_tests/ba_iterschur_suitesparse_clustjacobi_auto_test.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ziye01/CV_learning/ceres-solver/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/ba_iterschur_suitesparse_clustjacobi_auto_test.cc.o"
	cd /home/ziye01/CV_learning/ceres-solver/build/internal/ceres/generated_bundle_adjustment_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/ba_iterschur_suitesparse_clustjacobi_auto_test.cc.o -c /home/ziye01/CV_learning/ceres-solver/internal/ceres/generated_bundle_adjustment_tests/ba_iterschur_suitesparse_clustjacobi_auto_test.cc

internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/ba_iterschur_suitesparse_clustjacobi_auto_test.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/ba_iterschur_suitesparse_clustjacobi_auto_test.cc.i"
	cd /home/ziye01/CV_learning/ceres-solver/build/internal/ceres/generated_bundle_adjustment_tests && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ziye01/CV_learning/ceres-solver/internal/ceres/generated_bundle_adjustment_tests/ba_iterschur_suitesparse_clustjacobi_auto_test.cc > CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/ba_iterschur_suitesparse_clustjacobi_auto_test.cc.i

internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/ba_iterschur_suitesparse_clustjacobi_auto_test.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/ba_iterschur_suitesparse_clustjacobi_auto_test.cc.s"
	cd /home/ziye01/CV_learning/ceres-solver/build/internal/ceres/generated_bundle_adjustment_tests && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ziye01/CV_learning/ceres-solver/internal/ceres/generated_bundle_adjustment_tests/ba_iterschur_suitesparse_clustjacobi_auto_test.cc -o CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/ba_iterschur_suitesparse_clustjacobi_auto_test.cc.s

# Object files for target ba_iterschur_suitesparse_clustjacobi_auto_test
ba_iterschur_suitesparse_clustjacobi_auto_test_OBJECTS = \
"CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/ba_iterschur_suitesparse_clustjacobi_auto_test.cc.o"

# External object files for target ba_iterschur_suitesparse_clustjacobi_auto_test
ba_iterschur_suitesparse_clustjacobi_auto_test_EXTERNAL_OBJECTS =

bin/ba_iterschur_suitesparse_clustjacobi_auto_test: internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/ba_iterschur_suitesparse_clustjacobi_auto_test.cc.o
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/build.make
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: lib/libgtest.a
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: lib/libtest_util.a
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: lib/libceres.a
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/libspqr.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: lib/libgtest.a
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: lib/libceres.a
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/libspqr.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/libtbb.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/libglog.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/libcholmod.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/libamd.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/libcamd.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/libccolamd.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/libcolamd.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/local/cuda-11.0/lib64/libcudart_static.a
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/librt.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/local/cuda-11.0/lib64/libcublas.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/local/cuda-11.0/lib64/libcusolver.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/local/cuda-11.0/lib64/libcusparse.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: lib/libceres_cuda_kernels.a
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/liblapack.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/libblas.so
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
bin/ba_iterschur_suitesparse_clustjacobi_auto_test: internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ziye01/CV_learning/ceres-solver/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/ba_iterschur_suitesparse_clustjacobi_auto_test"
	cd /home/ziye01/CV_learning/ceres-solver/build/internal/ceres/generated_bundle_adjustment_tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/build: bin/ba_iterschur_suitesparse_clustjacobi_auto_test

.PHONY : internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/build

internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/clean:
	cd /home/ziye01/CV_learning/ceres-solver/build/internal/ceres/generated_bundle_adjustment_tests && $(CMAKE_COMMAND) -P CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/cmake_clean.cmake
.PHONY : internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/clean

internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/depend:
	cd /home/ziye01/CV_learning/ceres-solver/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ziye01/CV_learning/ceres-solver /home/ziye01/CV_learning/ceres-solver/internal/ceres/generated_bundle_adjustment_tests /home/ziye01/CV_learning/ceres-solver/build /home/ziye01/CV_learning/ceres-solver/build/internal/ceres/generated_bundle_adjustment_tests /home/ziye01/CV_learning/ceres-solver/build/internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : internal/ceres/generated_bundle_adjustment_tests/CMakeFiles/ba_iterschur_suitesparse_clustjacobi_auto_test.dir/depend

