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
CMAKE_SOURCE_DIR = /home/lees/linux_bak/LSLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lees/linux_bak/LSLAM/build

# Include any dependencies generated for this target.
include app/CMakeFiles/run_stereo.dir/depend.make

# Include the progress variables for this target.
include app/CMakeFiles/run_stereo.dir/progress.make

# Include the compile flags for this target's objects.
include app/CMakeFiles/run_stereo.dir/flags.make

app/CMakeFiles/run_stereo.dir/run_stereo.cpp.o: app/CMakeFiles/run_stereo.dir/flags.make
app/CMakeFiles/run_stereo.dir/run_stereo.cpp.o: ../app/run_stereo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lees/linux_bak/LSLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object app/CMakeFiles/run_stereo.dir/run_stereo.cpp.o"
	cd /home/lees/linux_bak/LSLAM/build/app && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_stereo.dir/run_stereo.cpp.o -c /home/lees/linux_bak/LSLAM/app/run_stereo.cpp

app/CMakeFiles/run_stereo.dir/run_stereo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_stereo.dir/run_stereo.cpp.i"
	cd /home/lees/linux_bak/LSLAM/build/app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lees/linux_bak/LSLAM/app/run_stereo.cpp > CMakeFiles/run_stereo.dir/run_stereo.cpp.i

app/CMakeFiles/run_stereo.dir/run_stereo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_stereo.dir/run_stereo.cpp.s"
	cd /home/lees/linux_bak/LSLAM/build/app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lees/linux_bak/LSLAM/app/run_stereo.cpp -o CMakeFiles/run_stereo.dir/run_stereo.cpp.s

# Object files for target run_stereo
run_stereo_OBJECTS = \
"CMakeFiles/run_stereo.dir/run_stereo.cpp.o"

# External object files for target run_stereo
run_stereo_EXTERNAL_OBJECTS =

../bin/run_stereo: app/CMakeFiles/run_stereo.dir/run_stereo.cpp.o
../bin/run_stereo: app/CMakeFiles/run_stereo.dir/build.make
../bin/run_stereo: ../lib/liblslam.so
../bin/run_stereo: /usr/local/lib/libopencv_gapi.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_highgui.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_ml.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_objdetect.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_photo.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_stitching.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_video.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_videoio.so.4.5.1
../bin/run_stereo: /usr/local/lib/libpango_glgeometry.so
../bin/run_stereo: /usr/local/lib/libpango_plot.so
../bin/run_stereo: /usr/local/lib/libpango_python.so
../bin/run_stereo: /usr/local/lib/libpango_scene.so
../bin/run_stereo: /usr/local/lib/libpango_tools.so
../bin/run_stereo: /usr/local/lib/libpango_video.so
../bin/run_stereo: /usr/lib/x86_64-linux-gnu/libgtest.a
../bin/run_stereo: /usr/lib/x86_64-linux-gnu/libgtest_main.a
../bin/run_stereo: /usr/lib/x86_64-linux-gnu/libglog.so
../bin/run_stereo: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
../bin/run_stereo: /usr/local/lib/libfmt.a
../bin/run_stereo: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../bin/run_stereo: /usr/local/lib/libopencv_calib3d.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_dnn.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_features2d.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_flann.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_imgcodecs.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_imgproc.so.4.5.1
../bin/run_stereo: /usr/local/lib/libopencv_core.so.4.5.1
../bin/run_stereo: /usr/local/lib/libpango_geometry.so
../bin/run_stereo: /usr/local/lib/libpango_display.so
../bin/run_stereo: /usr/local/lib/libpango_vars.so
../bin/run_stereo: /usr/local/lib/libpango_packetstream.so
../bin/run_stereo: /usr/local/lib/libpango_windowing.so
../bin/run_stereo: /usr/local/lib/libpango_opengl.so
../bin/run_stereo: /usr/local/lib/libpango_image.so
../bin/run_stereo: /usr/local/lib/libpango_core.so
../bin/run_stereo: /usr/lib/x86_64-linux-gnu/libGLEW.so
../bin/run_stereo: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../bin/run_stereo: /usr/lib/x86_64-linux-gnu/libGLX.so
../bin/run_stereo: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/run_stereo: /usr/local/lib/libtinyobj.so
../bin/run_stereo: app/CMakeFiles/run_stereo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lees/linux_bak/LSLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/run_stereo"
	cd /home/lees/linux_bak/LSLAM/build/app && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_stereo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
app/CMakeFiles/run_stereo.dir/build: ../bin/run_stereo

.PHONY : app/CMakeFiles/run_stereo.dir/build

app/CMakeFiles/run_stereo.dir/clean:
	cd /home/lees/linux_bak/LSLAM/build/app && $(CMAKE_COMMAND) -P CMakeFiles/run_stereo.dir/cmake_clean.cmake
.PHONY : app/CMakeFiles/run_stereo.dir/clean

app/CMakeFiles/run_stereo.dir/depend:
	cd /home/lees/linux_bak/LSLAM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lees/linux_bak/LSLAM /home/lees/linux_bak/LSLAM/app /home/lees/linux_bak/LSLAM/build /home/lees/linux_bak/LSLAM/build/app /home/lees/linux_bak/LSLAM/build/app/CMakeFiles/run_stereo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : app/CMakeFiles/run_stereo.dir/depend

