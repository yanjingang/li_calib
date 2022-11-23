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
CMAKE_SOURCE_DIR = /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/Pangolin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/build-pangolin

# Include any dependencies generated for this target.
include examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/depend.make

# Include the progress variables for this target.
include examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/progress.make

# Include the compile flags for this target's objects.
include examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/flags.make

examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/main.cpp.o: examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/flags.make
examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/main.cpp.o: /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/Pangolin/examples/HelloPangolinOffscreen/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/build-pangolin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/main.cpp.o"
	cd /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/build-pangolin/examples/HelloPangolinOffscreen && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/HelloPangolinOffscreen.dir/main.cpp.o -c /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/Pangolin/examples/HelloPangolinOffscreen/main.cpp

examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HelloPangolinOffscreen.dir/main.cpp.i"
	cd /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/build-pangolin/examples/HelloPangolinOffscreen && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/Pangolin/examples/HelloPangolinOffscreen/main.cpp > CMakeFiles/HelloPangolinOffscreen.dir/main.cpp.i

examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HelloPangolinOffscreen.dir/main.cpp.s"
	cd /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/build-pangolin/examples/HelloPangolinOffscreen && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/Pangolin/examples/HelloPangolinOffscreen/main.cpp -o CMakeFiles/HelloPangolinOffscreen.dir/main.cpp.s

# Object files for target HelloPangolinOffscreen
HelloPangolinOffscreen_OBJECTS = \
"CMakeFiles/HelloPangolinOffscreen.dir/main.cpp.o"

# External object files for target HelloPangolinOffscreen
HelloPangolinOffscreen_EXTERNAL_OBJECTS =

examples/HelloPangolinOffscreen/HelloPangolinOffscreen: examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/main.cpp.o
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/build.make
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: src/libpangolin.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libOpenGL.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libGLX.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libGLU.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libGLEW.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libEGL.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libSM.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libICE.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libX11.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libXext.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libOpenGL.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libGLX.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libGLU.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libGLEW.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libEGL.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libSM.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libICE.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libX11.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: /usr/lib/x86_64-linux-gnu/libXext.so
examples/HelloPangolinOffscreen/HelloPangolinOffscreen: examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/build-pangolin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable HelloPangolinOffscreen"
	cd /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/build-pangolin/examples/HelloPangolinOffscreen && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HelloPangolinOffscreen.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/build: examples/HelloPangolinOffscreen/HelloPangolinOffscreen

.PHONY : examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/build

examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/clean:
	cd /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/build-pangolin/examples/HelloPangolinOffscreen && $(CMAKE_COMMAND) -P CMakeFiles/HelloPangolinOffscreen.dir/cmake_clean.cmake
.PHONY : examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/clean

examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/depend:
	cd /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/build-pangolin && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/Pangolin /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/Pangolin/examples/HelloPangolinOffscreen /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/build-pangolin /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/build-pangolin/examples/HelloPangolinOffscreen /home/ros/catkin_li_calib_git/src/Li-Calib/thirdparty/build-pangolin/examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/HelloPangolinOffscreen/CMakeFiles/HelloPangolinOffscreen.dir/depend

