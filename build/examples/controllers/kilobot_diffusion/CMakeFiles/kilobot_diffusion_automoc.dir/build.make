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
CMAKE_SOURCE_DIR = /home/giladgar/argos-insectbot/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/giladgar/argos-insectbot/build

# Utility rule file for kilobot_diffusion_automoc.

# Include the progress variables for this target.
include examples/controllers/kilobot_diffusion/CMakeFiles/kilobot_diffusion_automoc.dir/progress.make

examples/controllers/kilobot_diffusion/CMakeFiles/kilobot_diffusion_automoc:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/giladgar/argos-insectbot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic moc for target kilobot_diffusion"
	cd /home/giladgar/argos-insectbot/build/examples/controllers/kilobot_diffusion && /usr/bin/cmake -E cmake_autogen /home/giladgar/argos-insectbot/build/examples/controllers/kilobot_diffusion/CMakeFiles/kilobot_diffusion_automoc.dir/ Release

kilobot_diffusion_automoc: examples/controllers/kilobot_diffusion/CMakeFiles/kilobot_diffusion_automoc
kilobot_diffusion_automoc: examples/controllers/kilobot_diffusion/CMakeFiles/kilobot_diffusion_automoc.dir/build.make

.PHONY : kilobot_diffusion_automoc

# Rule to build all files generated by this target.
examples/controllers/kilobot_diffusion/CMakeFiles/kilobot_diffusion_automoc.dir/build: kilobot_diffusion_automoc

.PHONY : examples/controllers/kilobot_diffusion/CMakeFiles/kilobot_diffusion_automoc.dir/build

examples/controllers/kilobot_diffusion/CMakeFiles/kilobot_diffusion_automoc.dir/clean:
	cd /home/giladgar/argos-insectbot/build/examples/controllers/kilobot_diffusion && $(CMAKE_COMMAND) -P CMakeFiles/kilobot_diffusion_automoc.dir/cmake_clean.cmake
.PHONY : examples/controllers/kilobot_diffusion/CMakeFiles/kilobot_diffusion_automoc.dir/clean

examples/controllers/kilobot_diffusion/CMakeFiles/kilobot_diffusion_automoc.dir/depend:
	cd /home/giladgar/argos-insectbot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/giladgar/argos-insectbot/src /home/giladgar/argos-insectbot/src/examples/controllers/kilobot_diffusion /home/giladgar/argos-insectbot/build /home/giladgar/argos-insectbot/build/examples/controllers/kilobot_diffusion /home/giladgar/argos-insectbot/build/examples/controllers/kilobot_diffusion/CMakeFiles/kilobot_diffusion_automoc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/controllers/kilobot_diffusion/CMakeFiles/kilobot_diffusion_automoc.dir/depend
