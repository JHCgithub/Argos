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

# Include any dependencies generated for this target.
include controllers/footbot_nn/CMakeFiles/footbot_nn.dir/depend.make

# Include the progress variables for this target.
include controllers/footbot_nn/CMakeFiles/footbot_nn.dir/progress.make

# Include the compile flags for this target's objects.
include controllers/footbot_nn/CMakeFiles/footbot_nn.dir/flags.make

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/footbot_nn_autogen/mocs_compilation.cpp.o: controllers/footbot_nn/CMakeFiles/footbot_nn.dir/flags.make
controllers/footbot_nn/CMakeFiles/footbot_nn.dir/footbot_nn_autogen/mocs_compilation.cpp.o: controllers/footbot_nn/footbot_nn_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/project/Desktop/argos/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object controllers/footbot_nn/CMakeFiles/footbot_nn.dir/footbot_nn_autogen/mocs_compilation.cpp.o"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_nn.dir/footbot_nn_autogen/mocs_compilation.cpp.o -c /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn/footbot_nn_autogen/mocs_compilation.cpp

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/footbot_nn_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_nn.dir/footbot_nn_autogen/mocs_compilation.cpp.i"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn/footbot_nn_autogen/mocs_compilation.cpp > CMakeFiles/footbot_nn.dir/footbot_nn_autogen/mocs_compilation.cpp.i

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/footbot_nn_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_nn.dir/footbot_nn_autogen/mocs_compilation.cpp.s"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn/footbot_nn_autogen/mocs_compilation.cpp -o CMakeFiles/footbot_nn.dir/footbot_nn_autogen/mocs_compilation.cpp.s

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/neural_network.cpp.o: controllers/footbot_nn/CMakeFiles/footbot_nn.dir/flags.make
controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/neural_network.cpp.o: ../controllers/footbot_nn/nn/neural_network.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/project/Desktop/argos/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/neural_network.cpp.o"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_nn.dir/nn/neural_network.cpp.o -c /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn/nn/neural_network.cpp

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/neural_network.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_nn.dir/nn/neural_network.cpp.i"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn/nn/neural_network.cpp > CMakeFiles/footbot_nn.dir/nn/neural_network.cpp.i

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/neural_network.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_nn.dir/nn/neural_network.cpp.s"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn/nn/neural_network.cpp -o CMakeFiles/footbot_nn.dir/nn/neural_network.cpp.s

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/perceptron.cpp.o: controllers/footbot_nn/CMakeFiles/footbot_nn.dir/flags.make
controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/perceptron.cpp.o: ../controllers/footbot_nn/nn/perceptron.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/project/Desktop/argos/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/perceptron.cpp.o"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_nn.dir/nn/perceptron.cpp.o -c /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn/nn/perceptron.cpp

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/perceptron.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_nn.dir/nn/perceptron.cpp.i"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn/nn/perceptron.cpp > CMakeFiles/footbot_nn.dir/nn/perceptron.cpp.i

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/perceptron.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_nn.dir/nn/perceptron.cpp.s"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn/nn/perceptron.cpp -o CMakeFiles/footbot_nn.dir/nn/perceptron.cpp.s

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/ctrnn_multilayer.cpp.o: controllers/footbot_nn/CMakeFiles/footbot_nn.dir/flags.make
controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/ctrnn_multilayer.cpp.o: ../controllers/footbot_nn/nn/ctrnn_multilayer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/project/Desktop/argos/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/ctrnn_multilayer.cpp.o"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_nn.dir/nn/ctrnn_multilayer.cpp.o -c /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn/nn/ctrnn_multilayer.cpp

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/ctrnn_multilayer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_nn.dir/nn/ctrnn_multilayer.cpp.i"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn/nn/ctrnn_multilayer.cpp > CMakeFiles/footbot_nn.dir/nn/ctrnn_multilayer.cpp.i

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/ctrnn_multilayer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_nn.dir/nn/ctrnn_multilayer.cpp.s"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn/nn/ctrnn_multilayer.cpp -o CMakeFiles/footbot_nn.dir/nn/ctrnn_multilayer.cpp.s

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/footbot_nn_controller.cpp.o: controllers/footbot_nn/CMakeFiles/footbot_nn.dir/flags.make
controllers/footbot_nn/CMakeFiles/footbot_nn.dir/footbot_nn_controller.cpp.o: ../controllers/footbot_nn/footbot_nn_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/project/Desktop/argos/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object controllers/footbot_nn/CMakeFiles/footbot_nn.dir/footbot_nn_controller.cpp.o"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_nn.dir/footbot_nn_controller.cpp.o -c /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn/footbot_nn_controller.cpp

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/footbot_nn_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_nn.dir/footbot_nn_controller.cpp.i"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn/footbot_nn_controller.cpp > CMakeFiles/footbot_nn.dir/footbot_nn_controller.cpp.i

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/footbot_nn_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_nn.dir/footbot_nn_controller.cpp.s"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn/footbot_nn_controller.cpp -o CMakeFiles/footbot_nn.dir/footbot_nn_controller.cpp.s

# Object files for target footbot_nn
footbot_nn_OBJECTS = \
"CMakeFiles/footbot_nn.dir/footbot_nn_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/footbot_nn.dir/nn/neural_network.cpp.o" \
"CMakeFiles/footbot_nn.dir/nn/perceptron.cpp.o" \
"CMakeFiles/footbot_nn.dir/nn/ctrnn_multilayer.cpp.o" \
"CMakeFiles/footbot_nn.dir/footbot_nn_controller.cpp.o"

# External object files for target footbot_nn
footbot_nn_EXTERNAL_OBJECTS =

controllers/footbot_nn/libfootbot_nn.so: controllers/footbot_nn/CMakeFiles/footbot_nn.dir/footbot_nn_autogen/mocs_compilation.cpp.o
controllers/footbot_nn/libfootbot_nn.so: controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/neural_network.cpp.o
controllers/footbot_nn/libfootbot_nn.so: controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/perceptron.cpp.o
controllers/footbot_nn/libfootbot_nn.so: controllers/footbot_nn/CMakeFiles/footbot_nn.dir/nn/ctrnn_multilayer.cpp.o
controllers/footbot_nn/libfootbot_nn.so: controllers/footbot_nn/CMakeFiles/footbot_nn.dir/footbot_nn_controller.cpp.o
controllers/footbot_nn/libfootbot_nn.so: controllers/footbot_nn/CMakeFiles/footbot_nn.dir/build.make
controllers/footbot_nn/libfootbot_nn.so: controllers/footbot_nn/CMakeFiles/footbot_nn.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/project/Desktop/argos/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared library libfootbot_nn.so"
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/footbot_nn.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
controllers/footbot_nn/CMakeFiles/footbot_nn.dir/build: controllers/footbot_nn/libfootbot_nn.so

.PHONY : controllers/footbot_nn/CMakeFiles/footbot_nn.dir/build

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/clean:
	cd /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn && $(CMAKE_COMMAND) -P CMakeFiles/footbot_nn.dir/cmake_clean.cmake
.PHONY : controllers/footbot_nn/CMakeFiles/footbot_nn.dir/clean

controllers/footbot_nn/CMakeFiles/footbot_nn.dir/depend:
	cd /home/project/Desktop/argos/argos3-examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/project/Desktop/argos/argos3-examples /home/project/Desktop/argos/argos3-examples/controllers/footbot_nn /home/project/Desktop/argos/argos3-examples/build /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn /home/project/Desktop/argos/argos3-examples/build/controllers/footbot_nn/CMakeFiles/footbot_nn.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controllers/footbot_nn/CMakeFiles/footbot_nn.dir/depend

