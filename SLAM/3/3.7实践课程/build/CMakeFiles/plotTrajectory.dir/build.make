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
CMAKE_SOURCE_DIR = /home/franka/home/CV_learning/SLAM/3/3.7实践课程

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/franka/home/CV_learning/SLAM/3/3.7实践课程/build

# Include any dependencies generated for this target.
include CMakeFiles/plotTrajectory.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/plotTrajectory.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/plotTrajectory.dir/flags.make

CMakeFiles/plotTrajectory.dir/src/plotTrajectory.cpp.o: CMakeFiles/plotTrajectory.dir/flags.make
CMakeFiles/plotTrajectory.dir/src/plotTrajectory.cpp.o: ../src/plotTrajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/franka/home/CV_learning/SLAM/3/3.7实践课程/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/plotTrajectory.dir/src/plotTrajectory.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/plotTrajectory.dir/src/plotTrajectory.cpp.o -c /home/franka/home/CV_learning/SLAM/3/3.7实践课程/src/plotTrajectory.cpp

CMakeFiles/plotTrajectory.dir/src/plotTrajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/plotTrajectory.dir/src/plotTrajectory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/franka/home/CV_learning/SLAM/3/3.7实践课程/src/plotTrajectory.cpp > CMakeFiles/plotTrajectory.dir/src/plotTrajectory.cpp.i

CMakeFiles/plotTrajectory.dir/src/plotTrajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/plotTrajectory.dir/src/plotTrajectory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/franka/home/CV_learning/SLAM/3/3.7实践课程/src/plotTrajectory.cpp -o CMakeFiles/plotTrajectory.dir/src/plotTrajectory.cpp.s

# Object files for target plotTrajectory
plotTrajectory_OBJECTS = \
"CMakeFiles/plotTrajectory.dir/src/plotTrajectory.cpp.o"

# External object files for target plotTrajectory
plotTrajectory_EXTERNAL_OBJECTS =

bin/plotTrajectory: CMakeFiles/plotTrajectory.dir/src/plotTrajectory.cpp.o
bin/plotTrajectory: CMakeFiles/plotTrajectory.dir/build.make
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_core.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_display.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_geometry.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_glgeometry.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_image.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_opengl.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_packetstream.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_plot.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_python.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_scene.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_tools.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_vars.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_video.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libpango_windowing.so
bin/plotTrajectory: /home/franka/home/CV_learning/Pangolin/devel/lib/libtinyobj.so
bin/plotTrajectory: CMakeFiles/plotTrajectory.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/franka/home/CV_learning/SLAM/3/3.7实践课程/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/plotTrajectory"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/plotTrajectory.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/plotTrajectory.dir/build: bin/plotTrajectory

.PHONY : CMakeFiles/plotTrajectory.dir/build

CMakeFiles/plotTrajectory.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/plotTrajectory.dir/cmake_clean.cmake
.PHONY : CMakeFiles/plotTrajectory.dir/clean

CMakeFiles/plotTrajectory.dir/depend:
	cd /home/franka/home/CV_learning/SLAM/3/3.7实践课程/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/franka/home/CV_learning/SLAM/3/3.7实践课程 /home/franka/home/CV_learning/SLAM/3/3.7实践课程 /home/franka/home/CV_learning/SLAM/3/3.7实践课程/build /home/franka/home/CV_learning/SLAM/3/3.7实践课程/build /home/franka/home/CV_learning/SLAM/3/3.7实践课程/build/CMakeFiles/plotTrajectory.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/plotTrajectory.dir/depend

