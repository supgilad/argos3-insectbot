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

# Utility rule file for orbit_star_automoc.

# Include the progress variables for this target.
include examples/behaviors/CMakeFiles/orbit_star_automoc.dir/progress.make

examples/behaviors/CMakeFiles/orbit_star_automoc:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/giladgar/argos3-kilobot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic moc for target orbit_star"
	cd /home/giladgar/argos3-kilobot/build/examples/behaviors && /usr/bin/cmake -E cmake_autogen /home/giladgar/argos3-kilobot/build/examples/behaviors/CMakeFiles/orbit_star_automoc.dir/ Release

orbit_star_automoc: examples/behaviors/CMakeFiles/orbit_star_automoc
orbit_star_automoc: examples/behaviors/CMakeFiles/orbit_star_automoc.dir/build.make

.PHONY : orbit_star_automoc

# Rule to build all files generated by this target.
examples/behaviors/CMakeFiles/orbit_star_automoc.dir/build: orbit_star_automoc

.PHONY : examples/behaviors/CMakeFiles/orbit_star_automoc.dir/build

examples/behaviors/CMakeFiles/orbit_star_automoc.dir/clean:
	cd /home/giladgar/argos3-kilobot/build/examples/behaviors && $(CMAKE_COMMAND) -P CMakeFiles/orbit_star_automoc.dir/cmake_clean.cmake
.PHONY : examples/behaviors/CMakeFiles/orbit_star_automoc.dir/clean

examples/behaviors/CMakeFiles/orbit_star_automoc.dir/depend:
	cd /home/giladgar/argos3-kilobot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/giladgar/argos3-kilobot/src /home/giladgar/argos3-kilobot/src/examples/behaviors /home/giladgar/argos3-kilobot/build /home/giladgar/argos3-kilobot/build/examples/behaviors /home/giladgar/argos3-kilobot/build/examples/behaviors/CMakeFiles/orbit_star_automoc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/behaviors/CMakeFiles/orbit_star_automoc.dir/depend

