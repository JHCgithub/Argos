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
CMAKE_SOURCE_DIR = /home/project/Desktop/argos/argos3-examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/project/Desktop/argos/argos3-examples/build

# Utility rule file for eyebot_flocking_autogen.

# Include the progress variables for this target.
include controllers/eyebot_flocking/CMakeFiles/eyebot_flocking_autogen.dir/progress.make

controllers/eyebot_flocking/CMakeFiles/eyebot_flocking_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/project/Desktop/argos/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC for target eyebot_flocking"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/eyebot_flocking && /usr/bin/cmake -E cmake_autogen /home/project/Desktop/argos/argos3-examples/build/controllers/eyebot_flocking/CMakeFiles/eyebot_flocking_autogen.dir/AutogenInfo.json Debug

eyebot_flocking_autogen: controllers/eyebot_flocking/CMakeFiles/eyebot_flocking_autogen
eyebot_flocking_autogen: controllers/eyebot_flocking/CMakeFiles/eyebot_flocking_autogen.dir/build.make

.PHONY : eyebot_flocking_autogen

# Rule to build all files generated by this target.
controllers/eyebot_flocking/CMakeFiles/eyebot_flocking_autogen.dir/build: eyebot_flocking_autogen

.PHONY : controllers/eyebot_flocking/CMakeFiles/eyebot_flocking_autogen.dir/build

controllers/eyebot_flocking/CMakeFiles/eyebot_flocking_autogen.dir/clean:
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/eyebot_flocking && $(CMAKE_COMMAND) -P CMakeFiles/eyebot_flocking_autogen.dir/cmake_clean.cmake
.PHONY : controllers/eyebot_flocking/CMakeFiles/eyebot_flocking_autogen.dir/clean

controllers/eyebot_flocking/CMakeFiles/eyebot_flocking_autogen.dir/depend:
	cd /home/project/Desktop/argos/argos3-examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/project/Desktop/argos/argos3-examples /home/project/Desktop/argos/argos3-examples/controllers/eyebot_flocking /home/project/Desktop/argos/argos3-examples/build /home/project/Desktop/argos/argos3-examples/build/controllers/eyebot_flocking /home/project/Desktop/argos/argos3-examples/build/controllers/eyebot_flocking/CMakeFiles/eyebot_flocking_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controllers/eyebot_flocking/CMakeFiles/eyebot_flocking_autogen.dir/depend

