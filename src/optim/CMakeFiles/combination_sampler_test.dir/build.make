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
include src/optim/CMakeFiles/combination_sampler_test.dir/depend.make

# Include the progress variables for this target.
include src/optim/CMakeFiles/combination_sampler_test.dir/progress.make

# Include the compile flags for this target's objects.
include src/optim/CMakeFiles/combination_sampler_test.dir/flags.make

src/optim/CMakeFiles/combination_sampler_test.dir/combination_sampler_test.cc.o: src/optim/CMakeFiles/combination_sampler_test.dir/flags.make
src/optim/CMakeFiles/combination_sampler_test.dir/combination_sampler_test.cc.o: src/optim/combination_sampler_test.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gy/DAGSfM_merge_v3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/optim/CMakeFiles/combination_sampler_test.dir/combination_sampler_test.cc.o"
	cd /home/gy/DAGSfM_merge_v3/src/optim && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/combination_sampler_test.dir/combination_sampler_test.cc.o -c /home/gy/DAGSfM_merge_v3/src/optim/combination_sampler_test.cc

src/optim/CMakeFiles/combination_sampler_test.dir/combination_sampler_test.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/combination_sampler_test.dir/combination_sampler_test.cc.i"
	cd /home/gy/DAGSfM_merge_v3/src/optim && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gy/DAGSfM_merge_v3/src/optim/combination_sampler_test.cc > CMakeFiles/combination_sampler_test.dir/combination_sampler_test.cc.i

src/optim/CMakeFiles/combination_sampler_test.dir/combination_sampler_test.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/combination_sampler_test.dir/combination_sampler_test.cc.s"
	cd /home/gy/DAGSfM_merge_v3/src/optim && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gy/DAGSfM_merge_v3/src/optim/combination_sampler_test.cc -o CMakeFiles/combination_sampler_test.dir/combination_sampler_test.cc.s

# Object files for target combination_sampler_test
combination_sampler_test_OBJECTS = \
"CMakeFiles/combination_sampler_test.dir/combination_sampler_test.cc.o"

# External object files for target combination_sampler_test
combination_sampler_test_EXTERNAL_OBJECTS =

src/optim/combination_sampler_test: src/optim/CMakeFiles/combination_sampler_test.dir/combination_sampler_test.cc.o
src/optim/combination_sampler_test: src/optim/CMakeFiles/combination_sampler_test.dir/build.make
src/optim/combination_sampler_test: src/libcolmap.a
src/optim/combination_sampler_test: lib/FLANN/libflann.a
src/optim/combination_sampler_test: lib/Graclus/libgraclus.a
src/optim/combination_sampler_test: lib/LSD/liblsd.a
src/optim/combination_sampler_test: lib/PBA/libpba.a
src/optim/combination_sampler_test: lib/SQLite/libsqlite3.a
src/optim/combination_sampler_test: lib/SiftGPU/libsift_gpu.a
src/optim/combination_sampler_test: lib/VLFeat/libvlfeat.a
src/optim/combination_sampler_test: src/libcolmap_cuda.a
src/optim/combination_sampler_test: src/libcolmap.a
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.a
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libboost_program_options.a
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libboost_regex.a
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libboost_system.a
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libglog.so
src/optim/combination_sampler_test: /usr/lib/libgtest_main.a
src/optim/combination_sampler_test: /usr/lib/libgtest.a
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libfreeimage.so
src/optim/combination_sampler_test: /usr/local/lib/libceres.a
src/optim/combination_sampler_test: /usr/local/lib/libigraph.so
src/optim/combination_sampler_test: /usr/local/lib/librpc.a
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libGL.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libGLU.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.9.5
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.9.5
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libCGAL.so.13.0.1
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libgmp.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.a
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libGLEW.so
src/optim/combination_sampler_test: /usr/local/cuda/lib64/libcudart_static.a
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libglog.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.1
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libspqr.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libcholmod.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libccolamd.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libcamd.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libcolamd.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libamd.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/liblapack.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libf77blas.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libatlas.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libcxsparse.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/liblapack.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libf77blas.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libatlas.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libcxsparse.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/librt.so
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5
src/optim/combination_sampler_test: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.9.5
src/optim/combination_sampler_test: src/optim/CMakeFiles/combination_sampler_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gy/DAGSfM_merge_v3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable combination_sampler_test"
	cd /home/gy/DAGSfM_merge_v3/src/optim && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/combination_sampler_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/optim/CMakeFiles/combination_sampler_test.dir/build: src/optim/combination_sampler_test

.PHONY : src/optim/CMakeFiles/combination_sampler_test.dir/build

src/optim/CMakeFiles/combination_sampler_test.dir/clean:
	cd /home/gy/DAGSfM_merge_v3/src/optim && $(CMAKE_COMMAND) -P CMakeFiles/combination_sampler_test.dir/cmake_clean.cmake
.PHONY : src/optim/CMakeFiles/combination_sampler_test.dir/clean

src/optim/CMakeFiles/combination_sampler_test.dir/depend:
	cd /home/gy/DAGSfM_merge_v3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gy/DAGSfM_merge_v3 /home/gy/DAGSfM_merge_v3/src/optim /home/gy/DAGSfM_merge_v3 /home/gy/DAGSfM_merge_v3/src/optim /home/gy/DAGSfM_merge_v3/src/optim/CMakeFiles/combination_sampler_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/optim/CMakeFiles/combination_sampler_test.dir/depend
