# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xjy/workspace/MPC_TrajectoryPlanning

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xjy/workspace/MPC_TrajectoryPlanning/build

# Include any dependencies generated for this target.
include data/CMakeFiles/data.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include data/CMakeFiles/data.dir/compiler_depend.make

# Include the progress variables for this target.
include data/CMakeFiles/data.dir/progress.make

# Include the compile flags for this target's objects.
include data/CMakeFiles/data.dir/flags.make

data/CMakeFiles/data.dir/savedata.cc.o: data/CMakeFiles/data.dir/flags.make
data/CMakeFiles/data.dir/savedata.cc.o: ../data/savedata.cc
data/CMakeFiles/data.dir/savedata.cc.o: data/CMakeFiles/data.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xjy/workspace/MPC_TrajectoryPlanning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object data/CMakeFiles/data.dir/savedata.cc.o"
	cd /home/xjy/workspace/MPC_TrajectoryPlanning/build/data && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT data/CMakeFiles/data.dir/savedata.cc.o -MF CMakeFiles/data.dir/savedata.cc.o.d -o CMakeFiles/data.dir/savedata.cc.o -c /home/xjy/workspace/MPC_TrajectoryPlanning/data/savedata.cc

data/CMakeFiles/data.dir/savedata.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/savedata.cc.i"
	cd /home/xjy/workspace/MPC_TrajectoryPlanning/build/data && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xjy/workspace/MPC_TrajectoryPlanning/data/savedata.cc > CMakeFiles/data.dir/savedata.cc.i

data/CMakeFiles/data.dir/savedata.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/savedata.cc.s"
	cd /home/xjy/workspace/MPC_TrajectoryPlanning/build/data && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xjy/workspace/MPC_TrajectoryPlanning/data/savedata.cc -o CMakeFiles/data.dir/savedata.cc.s

# Object files for target data
data_OBJECTS = \
"CMakeFiles/data.dir/savedata.cc.o"

# External object files for target data
data_EXTERNAL_OBJECTS =

data/libdata.a: data/CMakeFiles/data.dir/savedata.cc.o
data/libdata.a: data/CMakeFiles/data.dir/build.make
data/libdata.a: data/CMakeFiles/data.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xjy/workspace/MPC_TrajectoryPlanning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libdata.a"
	cd /home/xjy/workspace/MPC_TrajectoryPlanning/build/data && $(CMAKE_COMMAND) -P CMakeFiles/data.dir/cmake_clean_target.cmake
	cd /home/xjy/workspace/MPC_TrajectoryPlanning/build/data && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/data.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
data/CMakeFiles/data.dir/build: data/libdata.a
.PHONY : data/CMakeFiles/data.dir/build

data/CMakeFiles/data.dir/clean:
	cd /home/xjy/workspace/MPC_TrajectoryPlanning/build/data && $(CMAKE_COMMAND) -P CMakeFiles/data.dir/cmake_clean.cmake
.PHONY : data/CMakeFiles/data.dir/clean

data/CMakeFiles/data.dir/depend:
	cd /home/xjy/workspace/MPC_TrajectoryPlanning/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xjy/workspace/MPC_TrajectoryPlanning /home/xjy/workspace/MPC_TrajectoryPlanning/data /home/xjy/workspace/MPC_TrajectoryPlanning/build /home/xjy/workspace/MPC_TrajectoryPlanning/build/data /home/xjy/workspace/MPC_TrajectoryPlanning/build/data/CMakeFiles/data.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : data/CMakeFiles/data.dir/depend

