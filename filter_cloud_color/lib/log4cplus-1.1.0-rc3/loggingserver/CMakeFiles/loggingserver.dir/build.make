# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sibi/sandbox/sandbox444/filter_cloud_color

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sibi/sandbox/sandbox444/filter_cloud_color

# Include any dependencies generated for this target.
include lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/depend.make

# Include the progress variables for this target.
include lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/progress.make

# Include the compile flags for this target's objects.
include lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/flags.make

lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o: lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/flags.make
lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o: /home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/loggingserver/loggingserver.cxx
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sibi/sandbox/sandbox444/filter_cloud_color/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o"
	cd /home/sibi/sandbox/sandbox444/filter_cloud_color/lib/log4cplus-1.1.0-rc3/loggingserver && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/loggingserver.dir/loggingserver.cxx.o -c /home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/loggingserver/loggingserver.cxx

lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/loggingserver.dir/loggingserver.cxx.i"
	cd /home/sibi/sandbox/sandbox444/filter_cloud_color/lib/log4cplus-1.1.0-rc3/loggingserver && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/loggingserver/loggingserver.cxx > CMakeFiles/loggingserver.dir/loggingserver.cxx.i

lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/loggingserver.dir/loggingserver.cxx.s"
	cd /home/sibi/sandbox/sandbox444/filter_cloud_color/lib/log4cplus-1.1.0-rc3/loggingserver && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/loggingserver/loggingserver.cxx -o CMakeFiles/loggingserver.dir/loggingserver.cxx.s

lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o.requires:
.PHONY : lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o.requires

lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o.provides: lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o.requires
	$(MAKE) -f lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/build.make lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o.provides.build
.PHONY : lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o.provides

lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o.provides.build: lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o

# Object files for target loggingserver
loggingserver_OBJECTS = \
"CMakeFiles/loggingserver.dir/loggingserver.cxx.o"

# External object files for target loggingserver
loggingserver_EXTERNAL_OBJECTS =

3rdparty_bin/loggingserver: lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o
3rdparty_bin/loggingserver: lib/liblog4cplus.a
3rdparty_bin/loggingserver: lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/build.make
3rdparty_bin/loggingserver: lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../../3rdparty_bin/loggingserver"
	cd /home/sibi/sandbox/sandbox444/filter_cloud_color/lib/log4cplus-1.1.0-rc3/loggingserver && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/loggingserver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/build: 3rdparty_bin/loggingserver
.PHONY : lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/build

lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/requires: lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/loggingserver.cxx.o.requires
.PHONY : lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/requires

lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/clean:
	cd /home/sibi/sandbox/sandbox444/filter_cloud_color/lib/log4cplus-1.1.0-rc3/loggingserver && $(CMAKE_COMMAND) -P CMakeFiles/loggingserver.dir/cmake_clean.cmake
.PHONY : lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/clean

lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/depend:
	cd /home/sibi/sandbox/sandbox444/filter_cloud_color && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sibi/sandbox/sandbox444/filter_cloud_color /home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/loggingserver /home/sibi/sandbox/sandbox444/filter_cloud_color /home/sibi/sandbox/sandbox444/filter_cloud_color/lib/log4cplus-1.1.0-rc3/loggingserver /home/sibi/sandbox/sandbox444/filter_cloud_color/lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/log4cplus-1.1.0-rc3/loggingserver/CMakeFiles/loggingserver.dir/depend

