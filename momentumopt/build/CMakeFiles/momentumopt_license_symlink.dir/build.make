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

# Utility rule file for momentumopt_license_symlink.

# Include any custom commands dependencies for this target.
include CMakeFiles/momentumopt_license_symlink.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/momentumopt_license_symlink.dir/progress.make

CMakeFiles/momentumopt_license_symlink:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jhk/kino_dynamic_opt/momentumopt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Add the license.txt folder to the sphinx build."
	cd /home/jhk/kino_dynamic_opt/momentumopt/build/share/docs/sphinx && /home/jhk/.local/lib/python3.6/site-packages/cmake/data/bin/cmake -E create_symlink /home/jhk/kino_dynamic_opt/momentumopt/license.txt /home/jhk/kino_dynamic_opt/momentumopt/build/share/docs/sphinx/license.txt

momentumopt_license_symlink: CMakeFiles/momentumopt_license_symlink
momentumopt_license_symlink: CMakeFiles/momentumopt_license_symlink.dir/build.make
.PHONY : momentumopt_license_symlink

# Rule to build all files generated by this target.
CMakeFiles/momentumopt_license_symlink.dir/build: momentumopt_license_symlink
.PHONY : CMakeFiles/momentumopt_license_symlink.dir/build

CMakeFiles/momentumopt_license_symlink.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/momentumopt_license_symlink.dir/cmake_clean.cmake
.PHONY : CMakeFiles/momentumopt_license_symlink.dir/clean

CMakeFiles/momentumopt_license_symlink.dir/depend:
	cd /home/jhk/kino_dynamic_opt/momentumopt/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jhk/kino_dynamic_opt/momentumopt /home/jhk/kino_dynamic_opt/momentumopt /home/jhk/kino_dynamic_opt/momentumopt/build /home/jhk/kino_dynamic_opt/momentumopt/build /home/jhk/kino_dynamic_opt/momentumopt/build/CMakeFiles/momentumopt_license_symlink.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/momentumopt_license_symlink.dir/depend

