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
include src/CMakeFiles/aruco.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/aruco.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/aruco.dir/flags.make

src/CMakeFiles/aruco.dir/markerlabeler.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/markerlabeler.cpp.o: ../src/markerlabeler.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salinas/Libraries/aruco/aruco-2.0.6/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/aruco.dir/markerlabeler.cpp.o"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/markerlabeler.cpp.o -c /home/salinas/Libraries/aruco/aruco-2.0.6/src/markerlabeler.cpp

src/CMakeFiles/aruco.dir/markerlabeler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/markerlabeler.cpp.i"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salinas/Libraries/aruco/aruco-2.0.6/src/markerlabeler.cpp > CMakeFiles/aruco.dir/markerlabeler.cpp.i

src/CMakeFiles/aruco.dir/markerlabeler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/markerlabeler.cpp.s"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salinas/Libraries/aruco/aruco-2.0.6/src/markerlabeler.cpp -o CMakeFiles/aruco.dir/markerlabeler.cpp.s

src/CMakeFiles/aruco.dir/markerlabeler.cpp.o.requires:
.PHONY : src/CMakeFiles/aruco.dir/markerlabeler.cpp.o.requires

src/CMakeFiles/aruco.dir/markerlabeler.cpp.o.provides: src/CMakeFiles/aruco.dir/markerlabeler.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/markerlabeler.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/markerlabeler.cpp.o.provides

src/CMakeFiles/aruco.dir/markerlabeler.cpp.o.provides.build: src/CMakeFiles/aruco.dir/markerlabeler.cpp.o

src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o: ../src/cvdrawingutils.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salinas/Libraries/aruco/aruco-2.0.6/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/cvdrawingutils.cpp.o -c /home/salinas/Libraries/aruco/aruco-2.0.6/src/cvdrawingutils.cpp

src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/cvdrawingutils.cpp.i"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salinas/Libraries/aruco/aruco-2.0.6/src/cvdrawingutils.cpp > CMakeFiles/aruco.dir/cvdrawingutils.cpp.i

src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/cvdrawingutils.cpp.s"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salinas/Libraries/aruco/aruco-2.0.6/src/cvdrawingutils.cpp -o CMakeFiles/aruco.dir/cvdrawingutils.cpp.s

src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.requires:
.PHONY : src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.requires

src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.provides: src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.provides

src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.provides.build: src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o

src/CMakeFiles/aruco.dir/dictionary.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/dictionary.cpp.o: ../src/dictionary.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salinas/Libraries/aruco/aruco-2.0.6/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/aruco.dir/dictionary.cpp.o"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/dictionary.cpp.o -c /home/salinas/Libraries/aruco/aruco-2.0.6/src/dictionary.cpp

src/CMakeFiles/aruco.dir/dictionary.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/dictionary.cpp.i"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salinas/Libraries/aruco/aruco-2.0.6/src/dictionary.cpp > CMakeFiles/aruco.dir/dictionary.cpp.i

src/CMakeFiles/aruco.dir/dictionary.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/dictionary.cpp.s"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salinas/Libraries/aruco/aruco-2.0.6/src/dictionary.cpp -o CMakeFiles/aruco.dir/dictionary.cpp.s

src/CMakeFiles/aruco.dir/dictionary.cpp.o.requires:
.PHONY : src/CMakeFiles/aruco.dir/dictionary.cpp.o.requires

src/CMakeFiles/aruco.dir/dictionary.cpp.o.provides: src/CMakeFiles/aruco.dir/dictionary.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/dictionary.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/dictionary.cpp.o.provides

src/CMakeFiles/aruco.dir/dictionary.cpp.o.provides.build: src/CMakeFiles/aruco.dir/dictionary.cpp.o

src/CMakeFiles/aruco.dir/posetracker.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/posetracker.cpp.o: ../src/posetracker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salinas/Libraries/aruco/aruco-2.0.6/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/aruco.dir/posetracker.cpp.o"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/posetracker.cpp.o -c /home/salinas/Libraries/aruco/aruco-2.0.6/src/posetracker.cpp

src/CMakeFiles/aruco.dir/posetracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/posetracker.cpp.i"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salinas/Libraries/aruco/aruco-2.0.6/src/posetracker.cpp > CMakeFiles/aruco.dir/posetracker.cpp.i

src/CMakeFiles/aruco.dir/posetracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/posetracker.cpp.s"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salinas/Libraries/aruco/aruco-2.0.6/src/posetracker.cpp -o CMakeFiles/aruco.dir/posetracker.cpp.s

src/CMakeFiles/aruco.dir/posetracker.cpp.o.requires:
.PHONY : src/CMakeFiles/aruco.dir/posetracker.cpp.o.requires

src/CMakeFiles/aruco.dir/posetracker.cpp.o.provides: src/CMakeFiles/aruco.dir/posetracker.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/posetracker.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/posetracker.cpp.o.provides

src/CMakeFiles/aruco.dir/posetracker.cpp.o.provides.build: src/CMakeFiles/aruco.dir/posetracker.cpp.o

src/CMakeFiles/aruco.dir/ar_omp.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/ar_omp.cpp.o: ../src/ar_omp.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salinas/Libraries/aruco/aruco-2.0.6/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/aruco.dir/ar_omp.cpp.o"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/ar_omp.cpp.o -c /home/salinas/Libraries/aruco/aruco-2.0.6/src/ar_omp.cpp

src/CMakeFiles/aruco.dir/ar_omp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/ar_omp.cpp.i"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salinas/Libraries/aruco/aruco-2.0.6/src/ar_omp.cpp > CMakeFiles/aruco.dir/ar_omp.cpp.i

src/CMakeFiles/aruco.dir/ar_omp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/ar_omp.cpp.s"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salinas/Libraries/aruco/aruco-2.0.6/src/ar_omp.cpp -o CMakeFiles/aruco.dir/ar_omp.cpp.s

src/CMakeFiles/aruco.dir/ar_omp.cpp.o.requires:
.PHONY : src/CMakeFiles/aruco.dir/ar_omp.cpp.o.requires

src/CMakeFiles/aruco.dir/ar_omp.cpp.o.provides: src/CMakeFiles/aruco.dir/ar_omp.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/ar_omp.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/ar_omp.cpp.o.provides

src/CMakeFiles/aruco.dir/ar_omp.cpp.o.provides.build: src/CMakeFiles/aruco.dir/ar_omp.cpp.o

src/CMakeFiles/aruco.dir/marker.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/marker.cpp.o: ../src/marker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salinas/Libraries/aruco/aruco-2.0.6/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/aruco.dir/marker.cpp.o"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/marker.cpp.o -c /home/salinas/Libraries/aruco/aruco-2.0.6/src/marker.cpp

src/CMakeFiles/aruco.dir/marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/marker.cpp.i"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salinas/Libraries/aruco/aruco-2.0.6/src/marker.cpp > CMakeFiles/aruco.dir/marker.cpp.i

src/CMakeFiles/aruco.dir/marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/marker.cpp.s"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salinas/Libraries/aruco/aruco-2.0.6/src/marker.cpp -o CMakeFiles/aruco.dir/marker.cpp.s

src/CMakeFiles/aruco.dir/marker.cpp.o.requires:
.PHONY : src/CMakeFiles/aruco.dir/marker.cpp.o.requires

src/CMakeFiles/aruco.dir/marker.cpp.o.provides: src/CMakeFiles/aruco.dir/marker.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/marker.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/marker.cpp.o.provides

src/CMakeFiles/aruco.dir/marker.cpp.o.provides.build: src/CMakeFiles/aruco.dir/marker.cpp.o

src/CMakeFiles/aruco.dir/markermap.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/markermap.cpp.o: ../src/markermap.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salinas/Libraries/aruco/aruco-2.0.6/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/aruco.dir/markermap.cpp.o"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/markermap.cpp.o -c /home/salinas/Libraries/aruco/aruco-2.0.6/src/markermap.cpp

src/CMakeFiles/aruco.dir/markermap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/markermap.cpp.i"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salinas/Libraries/aruco/aruco-2.0.6/src/markermap.cpp > CMakeFiles/aruco.dir/markermap.cpp.i

src/CMakeFiles/aruco.dir/markermap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/markermap.cpp.s"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salinas/Libraries/aruco/aruco-2.0.6/src/markermap.cpp -o CMakeFiles/aruco.dir/markermap.cpp.s

src/CMakeFiles/aruco.dir/markermap.cpp.o.requires:
.PHONY : src/CMakeFiles/aruco.dir/markermap.cpp.o.requires

src/CMakeFiles/aruco.dir/markermap.cpp.o.provides: src/CMakeFiles/aruco.dir/markermap.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/markermap.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/markermap.cpp.o.provides

src/CMakeFiles/aruco.dir/markermap.cpp.o.provides.build: src/CMakeFiles/aruco.dir/markermap.cpp.o

src/CMakeFiles/aruco.dir/markerdetector.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/markerdetector.cpp.o: ../src/markerdetector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salinas/Libraries/aruco/aruco-2.0.6/build/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/aruco.dir/markerdetector.cpp.o"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/markerdetector.cpp.o -c /home/salinas/Libraries/aruco/aruco-2.0.6/src/markerdetector.cpp

src/CMakeFiles/aruco.dir/markerdetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/markerdetector.cpp.i"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salinas/Libraries/aruco/aruco-2.0.6/src/markerdetector.cpp > CMakeFiles/aruco.dir/markerdetector.cpp.i

src/CMakeFiles/aruco.dir/markerdetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/markerdetector.cpp.s"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salinas/Libraries/aruco/aruco-2.0.6/src/markerdetector.cpp -o CMakeFiles/aruco.dir/markerdetector.cpp.s

src/CMakeFiles/aruco.dir/markerdetector.cpp.o.requires:
.PHONY : src/CMakeFiles/aruco.dir/markerdetector.cpp.o.requires

src/CMakeFiles/aruco.dir/markerdetector.cpp.o.provides: src/CMakeFiles/aruco.dir/markerdetector.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/markerdetector.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/markerdetector.cpp.o.provides

src/CMakeFiles/aruco.dir/markerdetector.cpp.o.provides.build: src/CMakeFiles/aruco.dir/markerdetector.cpp.o

src/CMakeFiles/aruco.dir/cameraparameters.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/cameraparameters.cpp.o: ../src/cameraparameters.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salinas/Libraries/aruco/aruco-2.0.6/build/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/aruco.dir/cameraparameters.cpp.o"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/cameraparameters.cpp.o -c /home/salinas/Libraries/aruco/aruco-2.0.6/src/cameraparameters.cpp

src/CMakeFiles/aruco.dir/cameraparameters.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/cameraparameters.cpp.i"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salinas/Libraries/aruco/aruco-2.0.6/src/cameraparameters.cpp > CMakeFiles/aruco.dir/cameraparameters.cpp.i

src/CMakeFiles/aruco.dir/cameraparameters.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/cameraparameters.cpp.s"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salinas/Libraries/aruco/aruco-2.0.6/src/cameraparameters.cpp -o CMakeFiles/aruco.dir/cameraparameters.cpp.s

src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.requires:
.PHONY : src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.requires

src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.provides: src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.provides

src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.provides.build: src/CMakeFiles/aruco.dir/cameraparameters.cpp.o

src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o: ../src/markerlabelers/dictionary_based.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salinas/Libraries/aruco/aruco-2.0.6/build/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o -c /home/salinas/Libraries/aruco/aruco-2.0.6/src/markerlabelers/dictionary_based.cpp

src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.i"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salinas/Libraries/aruco/aruco-2.0.6/src/markerlabelers/dictionary_based.cpp > CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.i

src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.s"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salinas/Libraries/aruco/aruco-2.0.6/src/markerlabelers/dictionary_based.cpp -o CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.s

src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o.requires:
.PHONY : src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o.requires

src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o.provides: src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o.provides

src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o.provides.build: src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o

src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o: ../src/markerlabelers/svmmarkers.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salinas/Libraries/aruco/aruco-2.0.6/build/CMakeFiles $(CMAKE_PROGRESS_11)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o -c /home/salinas/Libraries/aruco/aruco-2.0.6/src/markerlabelers/svmmarkers.cpp

src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.i"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salinas/Libraries/aruco/aruco-2.0.6/src/markerlabelers/svmmarkers.cpp > CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.i

src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.s"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salinas/Libraries/aruco/aruco-2.0.6/src/markerlabelers/svmmarkers.cpp -o CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.s

src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o.requires:
.PHONY : src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o.requires

src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o.provides: src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o.provides

src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o.provides.build: src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o

# Object files for target aruco
aruco_OBJECTS = \
"CMakeFiles/aruco.dir/markerlabeler.cpp.o" \
"CMakeFiles/aruco.dir/cvdrawingutils.cpp.o" \
"CMakeFiles/aruco.dir/dictionary.cpp.o" \
"CMakeFiles/aruco.dir/posetracker.cpp.o" \
"CMakeFiles/aruco.dir/ar_omp.cpp.o" \
"CMakeFiles/aruco.dir/marker.cpp.o" \
"CMakeFiles/aruco.dir/markermap.cpp.o" \
"CMakeFiles/aruco.dir/markerdetector.cpp.o" \
"CMakeFiles/aruco.dir/cameraparameters.cpp.o" \
"CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o" \
"CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o"

# External object files for target aruco
aruco_EXTERNAL_OBJECTS =

src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/markerlabeler.cpp.o
src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o
src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/dictionary.cpp.o
src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/posetracker.cpp.o
src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/ar_omp.cpp.o
src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/marker.cpp.o
src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/markermap.cpp.o
src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/markerdetector.cpp.o
src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/cameraparameters.cpp.o
src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o
src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o
src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/build.make
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_videostab.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_video.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_ts.a
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_superres.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_stitching.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_photo.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_ocl.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_objdetect.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_nonfree.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_ml.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_legacy.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_imgproc.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_highgui.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_gpu.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_flann.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_features2d.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_core.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_contrib.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_calib3d.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_nonfree.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_ocl.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_gpu.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_photo.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_objdetect.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_legacy.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_video.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_ml.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_calib3d.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_features2d.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_highgui.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_imgproc.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_flann.so.2.4.13
src/libaruco.so.2.0.6: /home/salinas/dev_/lib/libopencv_core.so.2.4.13
src/libaruco.so.2.0.6: src/CMakeFiles/aruco.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libaruco.so"
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco.dir/link.txt --verbose=$(VERBOSE)
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && $(CMAKE_COMMAND) -E cmake_symlink_library libaruco.so.2.0.6 libaruco.so.2.0 libaruco.so

src/libaruco.so.2.0: src/libaruco.so.2.0.6

src/libaruco.so: src/libaruco.so.2.0.6

# Rule to build all files generated by this target.
src/CMakeFiles/aruco.dir/build: src/libaruco.so
.PHONY : src/CMakeFiles/aruco.dir/build

src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/markerlabeler.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/dictionary.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/posetracker.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/ar_omp.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/marker.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/markermap.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/markerdetector.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/markerlabelers/dictionary_based.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/markerlabelers/svmmarkers.cpp.o.requires
.PHONY : src/CMakeFiles/aruco.dir/requires

src/CMakeFiles/aruco.dir/clean:
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build/src && $(CMAKE_COMMAND) -P CMakeFiles/aruco.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/aruco.dir/clean

src/CMakeFiles/aruco.dir/depend:
	cd /home/salinas/Libraries/aruco/aruco-2.0.6/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/salinas/Libraries/aruco/aruco-2.0.6 /home/salinas/Libraries/aruco/aruco-2.0.6/src /home/salinas/Libraries/aruco/aruco-2.0.6/build /home/salinas/Libraries/aruco/aruco-2.0.6/build/src /home/salinas/Libraries/aruco/aruco-2.0.6/build/src/CMakeFiles/aruco.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/aruco.dir/depend

