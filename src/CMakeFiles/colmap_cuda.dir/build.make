# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gy/DAGSfM_merge_v3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gy/DAGSfM_merge_v3

# Include any dependencies generated for this target.
include src/CMakeFiles/colmap_cuda.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/colmap_cuda.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/colmap_cuda.dir/flags.make

src/CMakeFiles/colmap_cuda.dir/util/cuda.cc.o: src/CMakeFiles/colmap_cuda.dir/flags.make
src/CMakeFiles/colmap_cuda.dir/util/cuda.cc.o: src/util/cuda.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gy/DAGSfM_merge_v3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/colmap_cuda.dir/util/cuda.cc.o"
	cd /home/gy/DAGSfM_merge_v3/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/colmap_cuda.dir/util/cuda.cc.o -c /home/gy/DAGSfM_merge_v3/src/util/cuda.cc

src/CMakeFiles/colmap_cuda.dir/util/cuda.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/colmap_cuda.dir/util/cuda.cc.i"
	cd /home/gy/DAGSfM_merge_v3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gy/DAGSfM_merge_v3/src/util/cuda.cc > CMakeFiles/colmap_cuda.dir/util/cuda.cc.i

src/CMakeFiles/colmap_cuda.dir/util/cuda.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/colmap_cuda.dir/util/cuda.cc.s"
	cd /home/gy/DAGSfM_merge_v3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gy/DAGSfM_merge_v3/src/util/cuda.cc -o CMakeFiles/colmap_cuda.dir/util/cuda.cc.s

src/CMakeFiles/colmap_cuda.dir/util/cudacc.cc.o: src/CMakeFiles/colmap_cuda.dir/flags.make
src/CMakeFiles/colmap_cuda.dir/util/cudacc.cc.o: src/util/cudacc.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gy/DAGSfM_merge_v3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/colmap_cuda.dir/util/cudacc.cc.o"
	cd /home/gy/DAGSfM_merge_v3/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/colmap_cuda.dir/util/cudacc.cc.o -c /home/gy/DAGSfM_merge_v3/src/util/cudacc.cc

src/CMakeFiles/colmap_cuda.dir/util/cudacc.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/colmap_cuda.dir/util/cudacc.cc.i"
	cd /home/gy/DAGSfM_merge_v3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gy/DAGSfM_merge_v3/src/util/cudacc.cc > CMakeFiles/colmap_cuda.dir/util/cudacc.cc.i

src/CMakeFiles/colmap_cuda.dir/util/cudacc.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/colmap_cuda.dir/util/cudacc.cc.s"
	cd /home/gy/DAGSfM_merge_v3/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gy/DAGSfM_merge_v3/src/util/cudacc.cc -o CMakeFiles/colmap_cuda.dir/util/cudacc.cc.s

# Object files for target colmap_cuda
colmap_cuda_OBJECTS = \
"CMakeFiles/colmap_cuda.dir/util/cuda.cc.o" \
"CMakeFiles/colmap_cuda.dir/util/cudacc.cc.o"

# External object files for target colmap_cuda
colmap_cuda_EXTERNAL_OBJECTS =

src/libcolmap_cuda.a: src/CMakeFiles/colmap_cuda.dir/util/cuda.cc.o
src/libcolmap_cuda.a: src/CMakeFiles/colmap_cuda.dir/util/cudacc.cc.o
src/libcolmap_cuda.a: src/CMakeFiles/colmap_cuda.dir/build.make
src/libcolmap_cuda.a: src/CMakeFiles/colmap_cuda.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gy/DAGSfM_merge_v3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libcolmap_cuda.a"
	cd /home/gy/DAGSfM_merge_v3/src && $(CMAKE_COMMAND) -P CMakeFiles/colmap_cuda.dir/cmake_clean_target.cmake
	cd /home/gy/DAGSfM_merge_v3/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/colmap_cuda.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/colmap_cuda.dir/build: src/libcolmap_cuda.a

.PHONY : src/CMakeFiles/colmap_cuda.dir/build

src/CMakeFiles/colmap_cuda.dir/clean:
	cd /home/gy/DAGSfM_merge_v3/src && $(CMAKE_COMMAND) -P CMakeFiles/colmap_cuda.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/colmap_cuda.dir/clean

src/CMakeFiles/colmap_cuda.dir/depend:
	cd /home/gy/DAGSfM_merge_v3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gy/DAGSfM_merge_v3 /home/gy/DAGSfM_merge_v3/src /home/gy/DAGSfM_merge_v3 /home/gy/DAGSfM_merge_v3/src /home/gy/DAGSfM_merge_v3/src/CMakeFiles/colmap_cuda.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/colmap_cuda.dir/depend
