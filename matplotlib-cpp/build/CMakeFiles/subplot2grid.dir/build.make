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
CMAKE_SOURCE_DIR = /mnt/d/Project/CPP_Project/matplotlib-cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/d/Project/CPP_Project/matplotlib-cpp/build

# Include any dependencies generated for this target.
include CMakeFiles/subplot2grid.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/subplot2grid.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/subplot2grid.dir/flags.make

CMakeFiles/subplot2grid.dir/examples/subplot2grid.cpp.o: CMakeFiles/subplot2grid.dir/flags.make
CMakeFiles/subplot2grid.dir/examples/subplot2grid.cpp.o: ../examples/subplot2grid.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/d/Project/CPP_Project/matplotlib-cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/subplot2grid.dir/examples/subplot2grid.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/subplot2grid.dir/examples/subplot2grid.cpp.o -c /mnt/d/Project/CPP_Project/matplotlib-cpp/examples/subplot2grid.cpp

CMakeFiles/subplot2grid.dir/examples/subplot2grid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/subplot2grid.dir/examples/subplot2grid.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/d/Project/CPP_Project/matplotlib-cpp/examples/subplot2grid.cpp > CMakeFiles/subplot2grid.dir/examples/subplot2grid.cpp.i

CMakeFiles/subplot2grid.dir/examples/subplot2grid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/subplot2grid.dir/examples/subplot2grid.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/d/Project/CPP_Project/matplotlib-cpp/examples/subplot2grid.cpp -o CMakeFiles/subplot2grid.dir/examples/subplot2grid.cpp.s

# Object files for target subplot2grid
subplot2grid_OBJECTS = \
"CMakeFiles/subplot2grid.dir/examples/subplot2grid.cpp.o"

# External object files for target subplot2grid
subplot2grid_EXTERNAL_OBJECTS =

bin/subplot2grid: CMakeFiles/subplot2grid.dir/examples/subplot2grid.cpp.o
bin/subplot2grid: CMakeFiles/subplot2grid.dir/build.make
bin/subplot2grid: /usr/lib/x86_64-linux-gnu/libpython3.8.so
bin/subplot2grid: CMakeFiles/subplot2grid.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/d/Project/CPP_Project/matplotlib-cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/subplot2grid"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/subplot2grid.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/subplot2grid.dir/build: bin/subplot2grid

.PHONY : CMakeFiles/subplot2grid.dir/build

CMakeFiles/subplot2grid.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/subplot2grid.dir/cmake_clean.cmake
.PHONY : CMakeFiles/subplot2grid.dir/clean

CMakeFiles/subplot2grid.dir/depend:
	cd /mnt/d/Project/CPP_Project/matplotlib-cpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/d/Project/CPP_Project/matplotlib-cpp /mnt/d/Project/CPP_Project/matplotlib-cpp /mnt/d/Project/CPP_Project/matplotlib-cpp/build /mnt/d/Project/CPP_Project/matplotlib-cpp/build /mnt/d/Project/CPP_Project/matplotlib-cpp/build/CMakeFiles/subplot2grid.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/subplot2grid.dir/depend

