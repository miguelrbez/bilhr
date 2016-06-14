# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/salinas/Libraries/aruco/aruco-2.0.6

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/salinas/Libraries/aruco/aruco-2.0.6/build

# Include any dependencies generated for this target.
include utils_calibration/CMakeFiles/aruco_calibration.dir/depend.make

# Include the progress variables for this target.
include utils_calibration/CMakeFiles/aruco_calibration.dir/progress.make

# Include the compile flags for this target's objects.
include utils_calibration/CMakeFiles/aruco_calibration.dir/flags.make

utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o: utils_calibration/CMakeFiles/aruco_calibration.dir/flags.make
utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o: ../utils_calibration/aruco_calibration.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salinas/Libraries/aruco/aruco-2.0.6/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/utils_calibration && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o -c /home/salinas/Libraries/aruco/aruco-2.0.6/utils_calibration/aruco_calibration.cpp

utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.i"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/utils_calibration && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salinas/Libraries/aruco/aruco-2.0.6/utils_calibration/aruco_calibration.cpp > CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.i

utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.s"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/utils_calibration && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salinas/Libraries/aruco/aruco-2.0.6/utils_calibration/aruco_calibration.cpp -o CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.s

utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o.requires:
.PHONY : utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o.requires

utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o.provides: utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o.requires
	$(MAKE) -f utils_calibration/CMakeFiles/aruco_calibration.dir/build.make utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o.provides.build
.PHONY : utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o.provides

utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o.provides.build: utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o

# Object files for target aruco_calibration
aruco_calibration_OBJECTS = \
"CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o"

# External object files for target aruco_calibration
aruco_calibration_EXTERNAL_OBJECTS =

utils_calibration/aruco_calibration: utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o
utils_calibration/aruco_calibration: utils_calibration/CMakeFiles/aruco_calibration.dir/build.make
utils_calibration/aruco_calibration: src/libaruco.so.2.0.6
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_videostab.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_video.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_ts.a
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_superres.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_stitching.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_photo.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_ocl.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_objdetect.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_nonfree.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_ml.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_legacy.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_imgproc.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_highgui.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_gpu.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_flann.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_features2d.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_core.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_contrib.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_calib3d.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_nonfree.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_ocl.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_gpu.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_photo.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_objdetect.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_legacy.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_video.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_ml.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_calib3d.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_features2d.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_highgui.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_imgproc.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_flann.so.2.4.13
utils_calibration/aruco_calibration: /home/salinas/dev_/lib/libopencv_core.so.2.4.13
utils_calibration/aruco_calibration: utils_calibration/CMakeFiles/aruco_calibration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable aruco_calibration"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/utils_calibration && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco_calibration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utils_calibration/CMakeFiles/aruco_calibration.dir/build: utils_calibration/aruco_calibration
.PHONY : utils_calibration/CMakeFiles/aruco_calibration.dir/build

utils_calibration/CMakeFiles/aruco_calibration.dir/requires: utils_calibration/CMakeFiles/aruco_calibration.dir/aruco_calibration.cpp.o.requires
.PHONY : utils_calibration/CMakeFiles/aruco_calibration.dir/requires

utils_calibration/CMakeFiles/aruco_calibration.dir/clean:
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/utils_calibration && $(CMAKE_COMMAND) -P CMakeFiles/aruco_calibration.dir/cmake_clean.cmake
.PHONY : utils_calibration/CMakeFiles/aruco_calibration.dir/clean

utils_calibration/CMakeFiles/aruco_calibration.dir/depend:
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/salinas/Libraries/aruco/aruco-2.0.6 /home/salinas/Libraries/aruco/aruco-2.0.6/utils_calibration /home/salinas/Libraries/aruco/aruco-2.0.6/build /home/salinas/Libraries/aruco/aruco-2.0.6/build/utils_calibration /home/salinas/Libraries/aruco/aruco-2.0.6/build/utils_calibration/CMakeFiles/aruco_calibration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utils_calibration/CMakeFiles/aruco_calibration.dir/depend

