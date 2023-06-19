# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/puzzlebot/ws/UltralyticsAPI/tensorrtx/yolov5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/puzzlebot/ws/UltralyticsAPI/tensorrtx/yolov5/build

# Include any dependencies generated for this target.
include CMakeFiles/myplugins.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/myplugins.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/myplugins.dir/flags.make

CMakeFiles/myplugins.dir/plugin/yololayer.cu.o: CMakeFiles/myplugins.dir/flags.make
CMakeFiles/myplugins.dir/plugin/yololayer.cu.o: ../plugin/yololayer.cu
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/puzzlebot/ws/UltralyticsAPI/tensorrtx/yolov5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CUDA object CMakeFiles/myplugins.dir/plugin/yololayer.cu.o"
	/usr/local/cuda/bin/nvcc  $(CUDA_DEFINES) $(CUDA_INCLUDES) $(CUDA_FLAGS) -x cu -c /home/puzzlebot/ws/UltralyticsAPI/tensorrtx/yolov5/plugin/yololayer.cu -o CMakeFiles/myplugins.dir/plugin/yololayer.cu.o

CMakeFiles/myplugins.dir/plugin/yololayer.cu.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CUDA source to CMakeFiles/myplugins.dir/plugin/yololayer.cu.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CUDA_CREATE_PREPROCESSED_SOURCE

CMakeFiles/myplugins.dir/plugin/yololayer.cu.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CUDA source to assembly CMakeFiles/myplugins.dir/plugin/yololayer.cu.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CUDA_CREATE_ASSEMBLY_SOURCE

CMakeFiles/myplugins.dir/plugin/yololayer.cu.o.requires:

.PHONY : CMakeFiles/myplugins.dir/plugin/yololayer.cu.o.requires

CMakeFiles/myplugins.dir/plugin/yololayer.cu.o.provides: CMakeFiles/myplugins.dir/plugin/yololayer.cu.o.requires
	$(MAKE) -f CMakeFiles/myplugins.dir/build.make CMakeFiles/myplugins.dir/plugin/yololayer.cu.o.provides.build
.PHONY : CMakeFiles/myplugins.dir/plugin/yololayer.cu.o.provides

CMakeFiles/myplugins.dir/plugin/yololayer.cu.o.provides.build: CMakeFiles/myplugins.dir/plugin/yololayer.cu.o


# Object files for target myplugins
myplugins_OBJECTS = \
"CMakeFiles/myplugins.dir/plugin/yololayer.cu.o"

# External object files for target myplugins
myplugins_EXTERNAL_OBJECTS =

CMakeFiles/myplugins.dir/cmake_device_link.o: CMakeFiles/myplugins.dir/plugin/yololayer.cu.o
CMakeFiles/myplugins.dir/cmake_device_link.o: CMakeFiles/myplugins.dir/build.make
CMakeFiles/myplugins.dir/cmake_device_link.o: CMakeFiles/myplugins.dir/dlink.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/puzzlebot/ws/UltralyticsAPI/tensorrtx/yolov5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CUDA device code CMakeFiles/myplugins.dir/cmake_device_link.o"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myplugins.dir/dlink.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/myplugins.dir/build: CMakeFiles/myplugins.dir/cmake_device_link.o

.PHONY : CMakeFiles/myplugins.dir/build

# Object files for target myplugins
myplugins_OBJECTS = \
"CMakeFiles/myplugins.dir/plugin/yololayer.cu.o"

# External object files for target myplugins
myplugins_EXTERNAL_OBJECTS =

libmyplugins.so: CMakeFiles/myplugins.dir/plugin/yololayer.cu.o
libmyplugins.so: CMakeFiles/myplugins.dir/build.make
libmyplugins.so: CMakeFiles/myplugins.dir/cmake_device_link.o
libmyplugins.so: CMakeFiles/myplugins.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/puzzlebot/ws/UltralyticsAPI/tensorrtx/yolov5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CUDA shared library libmyplugins.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myplugins.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/myplugins.dir/build: libmyplugins.so

.PHONY : CMakeFiles/myplugins.dir/build

CMakeFiles/myplugins.dir/requires: CMakeFiles/myplugins.dir/plugin/yololayer.cu.o.requires

.PHONY : CMakeFiles/myplugins.dir/requires

CMakeFiles/myplugins.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/myplugins.dir/cmake_clean.cmake
.PHONY : CMakeFiles/myplugins.dir/clean

CMakeFiles/myplugins.dir/depend:
	cd /home/puzzlebot/ws/UltralyticsAPI/tensorrtx/yolov5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/puzzlebot/ws/UltralyticsAPI/tensorrtx/yolov5 /home/puzzlebot/ws/UltralyticsAPI/tensorrtx/yolov5 /home/puzzlebot/ws/UltralyticsAPI/tensorrtx/yolov5/build /home/puzzlebot/ws/UltralyticsAPI/tensorrtx/yolov5/build /home/puzzlebot/ws/UltralyticsAPI/tensorrtx/yolov5/build/CMakeFiles/myplugins.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/myplugins.dir/depend

