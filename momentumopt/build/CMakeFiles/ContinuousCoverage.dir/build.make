# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /home/jhk/.local/lib/python3.6/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/jhk/.local/lib/python3.6/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jhk/kino_dynamic_opt/momentumopt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jhk/kino_dynamic_opt/momentumopt/build

# Utility rule file for ContinuousCoverage.

# Include any custom commands dependencies for this target.
include CMakeFiles/ContinuousCoverage.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ContinuousCoverage.dir/progress.make

CMakeFiles/ContinuousCoverage:
	/home/jhk/.local/lib/python3.6/site-packages/cmake/data/bin/ctest -D ContinuousCoverage

ContinuousCoverage: CMakeFiles/ContinuousCoverage
ContinuousCoverage: CMakeFiles/ContinuousCoverage.dir/build.make
.PHONY : ContinuousCoverage

# Rule to build all files generated by this target.
CMakeFiles/ContinuousCoverage.dir/build: ContinuousCoverage
.PHONY : CMakeFiles/ContinuousCoverage.dir/build

CMakeFiles/ContinuousCoverage.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ContinuousCoverage.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ContinuousCoverage.dir/clean

CMakeFiles/ContinuousCoverage.dir/depend:
	cd /home/jhk/kino_dynamic_opt/momentumopt/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jhk/kino_dynamic_opt/momentumopt /home/jhk/kino_dynamic_opt/momentumopt /home/jhk/kino_dynamic_opt/momentumopt/build /home/jhk/kino_dynamic_opt/momentumopt/build /home/jhk/kino_dynamic_opt/momentumopt/build/CMakeFiles/ContinuousCoverage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ContinuousCoverage.dir/depend

