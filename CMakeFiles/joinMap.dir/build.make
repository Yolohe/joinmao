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
CMAKE_SOURCE_DIR = /home/y2/Desktop/slam/realsenseslam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/y2/Desktop/slam/realsenseslam

# Include any dependencies generated for this target.
include CMakeFiles/joinMap.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/joinMap.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/joinMap.dir/flags.make

CMakeFiles/joinMap.dir/joinMap.o: CMakeFiles/joinMap.dir/flags.make
CMakeFiles/joinMap.dir/joinMap.o: joinMap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/y2/Desktop/slam/realsenseslam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/joinMap.dir/joinMap.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joinMap.dir/joinMap.o -c /home/y2/Desktop/slam/realsenseslam/joinMap.cpp

CMakeFiles/joinMap.dir/joinMap.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joinMap.dir/joinMap.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/y2/Desktop/slam/realsenseslam/joinMap.cpp > CMakeFiles/joinMap.dir/joinMap.i

CMakeFiles/joinMap.dir/joinMap.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joinMap.dir/joinMap.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/y2/Desktop/slam/realsenseslam/joinMap.cpp -o CMakeFiles/joinMap.dir/joinMap.s

# Object files for target joinMap
joinMap_OBJECTS = \
"CMakeFiles/joinMap.dir/joinMap.o"

# External object files for target joinMap
joinMap_EXTERNAL_OBJECTS =

joinMap: CMakeFiles/joinMap.dir/joinMap.o
joinMap: CMakeFiles/joinMap.dir/build.make
joinMap: /usr/local/lib/libopencv_gapi.so.4.8.0
joinMap: /usr/local/lib/libopencv_highgui.so.4.8.0
joinMap: /usr/local/lib/libopencv_ml.so.4.8.0
joinMap: /usr/local/lib/libopencv_objdetect.so.4.8.0
joinMap: /usr/local/lib/libopencv_photo.so.4.8.0
joinMap: /usr/local/lib/libopencv_stitching.so.4.8.0
joinMap: /usr/local/lib/libopencv_video.so.4.8.0
joinMap: /usr/local/lib/libopencv_videoio.so.4.8.0
joinMap: /usr/local/lib/libpangolin.so
joinMap: /usr/lib/x86_64-linux-gnu/librealsense2.so.2.54.2
joinMap: /usr/local/lib/libopencv_imgcodecs.so.4.8.0
joinMap: /usr/local/lib/libopencv_dnn.so.4.8.0
joinMap: /usr/local/lib/libopencv_calib3d.so.4.8.0
joinMap: /usr/local/lib/libopencv_features2d.so.4.8.0
joinMap: /usr/local/lib/libopencv_flann.so.4.8.0
joinMap: /usr/local/lib/libopencv_imgproc.so.4.8.0
joinMap: /usr/local/lib/libopencv_core.so.4.8.0
joinMap: /usr/lib/x86_64-linux-gnu/libGL.so
joinMap: /usr/lib/x86_64-linux-gnu/libGLU.so
joinMap: /usr/lib/x86_64-linux-gnu/libGLEW.so
joinMap: /usr/lib/x86_64-linux-gnu/libSM.so
joinMap: /usr/lib/x86_64-linux-gnu/libICE.so
joinMap: /usr/lib/x86_64-linux-gnu/libX11.so
joinMap: /usr/lib/x86_64-linux-gnu/libXext.so
joinMap: /usr/lib/x86_64-linux-gnu/libpython3.8.so
joinMap: /usr/lib/x86_64-linux-gnu/libdc1394.so
joinMap: /usr/lib/libOpenNI.so
joinMap: /usr/lib/libOpenNI2.so
joinMap: /usr/lib/x86_64-linux-gnu/libpng.so
joinMap: /usr/lib/x86_64-linux-gnu/libz.so
joinMap: /usr/lib/x86_64-linux-gnu/libjpeg.so
joinMap: /usr/lib/x86_64-linux-gnu/libtiff.so
joinMap: /usr/lib/x86_64-linux-gnu/libIlmImf.so
joinMap: /usr/lib/x86_64-linux-gnu/librsutils.a
joinMap: CMakeFiles/joinMap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/y2/Desktop/slam/realsenseslam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable joinMap"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/joinMap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/joinMap.dir/build: joinMap

.PHONY : CMakeFiles/joinMap.dir/build

CMakeFiles/joinMap.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/joinMap.dir/cmake_clean.cmake
.PHONY : CMakeFiles/joinMap.dir/clean

CMakeFiles/joinMap.dir/depend:
	cd /home/y2/Desktop/slam/realsenseslam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/y2/Desktop/slam/realsenseslam /home/y2/Desktop/slam/realsenseslam /home/y2/Desktop/slam/realsenseslam /home/y2/Desktop/slam/realsenseslam /home/y2/Desktop/slam/realsenseslam/CMakeFiles/joinMap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/joinMap.dir/depend

