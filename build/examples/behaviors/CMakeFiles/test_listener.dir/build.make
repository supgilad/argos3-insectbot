# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/giladgar/argos3-kilobot/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/giladgar/argos3-kilobot/build

# Include any dependencies generated for this target.
include examples/behaviors/CMakeFiles/test_listener.dir/depend.make

# Include the progress variables for this target.
include examples/behaviors/CMakeFiles/test_listener.dir/progress.make

# Include the compile flags for this target's objects.
include examples/behaviors/CMakeFiles/test_listener.dir/flags.make

examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o: examples/behaviors/CMakeFiles/test_listener.dir/flags.make
examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o: /home/giladgar/argos3-kilobot/src/examples/behaviors/test_listener.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giladgar/argos3-kilobot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o"
	cd /home/giladgar/argos3-kilobot/build/examples/behaviors && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/test_listener.dir/test_listener.c.o   -c /home/giladgar/argos3-kilobot/src/examples/behaviors/test_listener.c

examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/test_listener.dir/test_listener.c.i"
	cd /home/giladgar/argos3-kilobot/build/examples/behaviors && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/giladgar/argos3-kilobot/src/examples/behaviors/test_listener.c > CMakeFiles/test_listener.dir/test_listener.c.i

examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/test_listener.dir/test_listener.c.s"
	cd /home/giladgar/argos3-kilobot/build/examples/behaviors && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/giladgar/argos3-kilobot/src/examples/behaviors/test_listener.c -o CMakeFiles/test_listener.dir/test_listener.c.s

examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o.requires:

.PHONY : examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o.requires

examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o.provides: examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o.requires
	$(MAKE) -f examples/behaviors/CMakeFiles/test_listener.dir/build.make examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o.provides.build
.PHONY : examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o.provides

examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o.provides.build: examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o


examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o: examples/behaviors/CMakeFiles/test_listener.dir/flags.make
examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o: examples/behaviors/test_listener_automoc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giladgar/argos3-kilobot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o"
	cd /home/giladgar/argos3-kilobot/build/examples/behaviors && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o -c /home/giladgar/argos3-kilobot/build/examples/behaviors/test_listener_automoc.cpp

examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_listener.dir/test_listener_automoc.cpp.i"
	cd /home/giladgar/argos3-kilobot/build/examples/behaviors && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giladgar/argos3-kilobot/build/examples/behaviors/test_listener_automoc.cpp > CMakeFiles/test_listener.dir/test_listener_automoc.cpp.i

examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_listener.dir/test_listener_automoc.cpp.s"
	cd /home/giladgar/argos3-kilobot/build/examples/behaviors && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giladgar/argos3-kilobot/build/examples/behaviors/test_listener_automoc.cpp -o CMakeFiles/test_listener.dir/test_listener_automoc.cpp.s

examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o.requires:

.PHONY : examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o.requires

examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o.provides: examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o.requires
	$(MAKE) -f examples/behaviors/CMakeFiles/test_listener.dir/build.make examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o.provides.build
.PHONY : examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o.provides

examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o.provides.build: examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o


# Object files for target test_listener
test_listener_OBJECTS = \
"CMakeFiles/test_listener.dir/test_listener.c.o" \
"CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o"

# External object files for target test_listener
test_listener_EXTERNAL_OBJECTS =

examples/behaviors/test_listener: examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o
examples/behaviors/test_listener: examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o
examples/behaviors/test_listener: examples/behaviors/CMakeFiles/test_listener.dir/build.make
examples/behaviors/test_listener: plugins/robots/kilobot/libargos3plugin_simulator_kilolib.a
examples/behaviors/test_listener: /usr/lib/x86_64-linux-gnu/librt.so
examples/behaviors/test_listener: examples/behaviors/CMakeFiles/test_listener.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/giladgar/argos3-kilobot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable test_listener"
	cd /home/giladgar/argos3-kilobot/build/examples/behaviors && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_listener.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/behaviors/CMakeFiles/test_listener.dir/build: examples/behaviors/test_listener

.PHONY : examples/behaviors/CMakeFiles/test_listener.dir/build

examples/behaviors/CMakeFiles/test_listener.dir/requires: examples/behaviors/CMakeFiles/test_listener.dir/test_listener.c.o.requires
examples/behaviors/CMakeFiles/test_listener.dir/requires: examples/behaviors/CMakeFiles/test_listener.dir/test_listener_automoc.cpp.o.requires

.PHONY : examples/behaviors/CMakeFiles/test_listener.dir/requires

examples/behaviors/CMakeFiles/test_listener.dir/clean:
	cd /home/giladgar/argos3-kilobot/build/examples/behaviors && $(CMAKE_COMMAND) -P CMakeFiles/test_listener.dir/cmake_clean.cmake
.PHONY : examples/behaviors/CMakeFiles/test_listener.dir/clean

examples/behaviors/CMakeFiles/test_listener.dir/depend:
	cd /home/giladgar/argos3-kilobot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/giladgar/argos3-kilobot/src /home/giladgar/argos3-kilobot/src/examples/behaviors /home/giladgar/argos3-kilobot/build /home/giladgar/argos3-kilobot/build/examples/behaviors /home/giladgar/argos3-kilobot/build/examples/behaviors/CMakeFiles/test_listener.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/behaviors/CMakeFiles/test_listener.dir/depend

