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
CMAKE_SOURCE_DIR = /home/zgc/mr_ws/src/Stage

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zgc/mr_ws/build/Stage

# Include any dependencies generated for this target.
include examples/ctrl/CMakeFiles/lasernoise.dir/depend.make

# Include the progress variables for this target.
include examples/ctrl/CMakeFiles/lasernoise.dir/progress.make

# Include the compile flags for this target's objects.
include examples/ctrl/CMakeFiles/lasernoise.dir/flags.make

examples/ctrl/CMakeFiles/lasernoise.dir/lasernoise.cc.o: examples/ctrl/CMakeFiles/lasernoise.dir/flags.make
examples/ctrl/CMakeFiles/lasernoise.dir/lasernoise.cc.o: /home/zgc/mr_ws/src/Stage/examples/ctrl/lasernoise.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zgc/mr_ws/build/Stage/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/ctrl/CMakeFiles/lasernoise.dir/lasernoise.cc.o"
	cd /home/zgc/mr_ws/build/Stage/examples/ctrl && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lasernoise.dir/lasernoise.cc.o -c /home/zgc/mr_ws/src/Stage/examples/ctrl/lasernoise.cc

examples/ctrl/CMakeFiles/lasernoise.dir/lasernoise.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lasernoise.dir/lasernoise.cc.i"
	cd /home/zgc/mr_ws/build/Stage/examples/ctrl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zgc/mr_ws/src/Stage/examples/ctrl/lasernoise.cc > CMakeFiles/lasernoise.dir/lasernoise.cc.i

examples/ctrl/CMakeFiles/lasernoise.dir/lasernoise.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lasernoise.dir/lasernoise.cc.s"
	cd /home/zgc/mr_ws/build/Stage/examples/ctrl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zgc/mr_ws/src/Stage/examples/ctrl/lasernoise.cc -o CMakeFiles/lasernoise.dir/lasernoise.cc.s

# Object files for target lasernoise
lasernoise_OBJECTS = \
"CMakeFiles/lasernoise.dir/lasernoise.cc.o"

# External object files for target lasernoise
lasernoise_EXTERNAL_OBJECTS =

examples/ctrl/lasernoise.so: examples/ctrl/CMakeFiles/lasernoise.dir/lasernoise.cc.o
examples/ctrl/lasernoise.so: examples/ctrl/CMakeFiles/lasernoise.dir/build.make
examples/ctrl/lasernoise.so: libstage/libstage.so.4.3.0
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libOpenGL.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libGLX.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libGLU.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libltdl.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libjpeg.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libpng.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libz.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libfltk_images.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libfltk_forms.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libfltk_gl.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libfltk.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libSM.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libICE.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libX11.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libXext.so
examples/ctrl/lasernoise.so: /usr/lib/x86_64-linux-gnu/libm.so
examples/ctrl/lasernoise.so: examples/ctrl/CMakeFiles/lasernoise.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zgc/mr_ws/build/Stage/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared module lasernoise.so"
	cd /home/zgc/mr_ws/build/Stage/examples/ctrl && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lasernoise.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/ctrl/CMakeFiles/lasernoise.dir/build: examples/ctrl/lasernoise.so

.PHONY : examples/ctrl/CMakeFiles/lasernoise.dir/build

examples/ctrl/CMakeFiles/lasernoise.dir/clean:
	cd /home/zgc/mr_ws/build/Stage/examples/ctrl && $(CMAKE_COMMAND) -P CMakeFiles/lasernoise.dir/cmake_clean.cmake
.PHONY : examples/ctrl/CMakeFiles/lasernoise.dir/clean

examples/ctrl/CMakeFiles/lasernoise.dir/depend:
	cd /home/zgc/mr_ws/build/Stage && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zgc/mr_ws/src/Stage /home/zgc/mr_ws/src/Stage/examples/ctrl /home/zgc/mr_ws/build/Stage /home/zgc/mr_ws/build/Stage/examples/ctrl /home/zgc/mr_ws/build/Stage/examples/ctrl/CMakeFiles/lasernoise.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/ctrl/CMakeFiles/lasernoise.dir/depend
