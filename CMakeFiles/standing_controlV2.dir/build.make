# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/orl/Tianze_WS/Test_Control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/orl/Tianze_WS/Test_Control

# Include any dependencies generated for this target.
include CMakeFiles/standing_controlV2.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/standing_controlV2.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/standing_controlV2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/standing_controlV2.dir/flags.make

CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.o: CMakeFiles/standing_controlV2.dir/flags.make
CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.o: standing_controlV2.cpp
CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.o: CMakeFiles/standing_controlV2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.o -MF CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.o.d -o CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.o -c /home/orl/Tianze_WS/Test_Control/standing_controlV2.cpp

CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/orl/Tianze_WS/Test_Control/standing_controlV2.cpp > CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.i

CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/orl/Tianze_WS/Test_Control/standing_controlV2.cpp -o CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.s

CMakeFiles/standing_controlV2.dir/lowlevelapi.c.o: CMakeFiles/standing_controlV2.dir/flags.make
CMakeFiles/standing_controlV2.dir/lowlevelapi.c.o: lowlevelapi.c
CMakeFiles/standing_controlV2.dir/lowlevelapi.c.o: CMakeFiles/standing_controlV2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/standing_controlV2.dir/lowlevelapi.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/standing_controlV2.dir/lowlevelapi.c.o -MF CMakeFiles/standing_controlV2.dir/lowlevelapi.c.o.d -o CMakeFiles/standing_controlV2.dir/lowlevelapi.c.o -c /home/orl/Tianze_WS/Test_Control/lowlevelapi.c

CMakeFiles/standing_controlV2.dir/lowlevelapi.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/standing_controlV2.dir/lowlevelapi.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/orl/Tianze_WS/Test_Control/lowlevelapi.c > CMakeFiles/standing_controlV2.dir/lowlevelapi.c.i

CMakeFiles/standing_controlV2.dir/lowlevelapi.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/standing_controlV2.dir/lowlevelapi.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/orl/Tianze_WS/Test_Control/lowlevelapi.c -o CMakeFiles/standing_controlV2.dir/lowlevelapi.c.s

CMakeFiles/standing_controlV2.dir/libartl/artl.c.o: CMakeFiles/standing_controlV2.dir/flags.make
CMakeFiles/standing_controlV2.dir/libartl/artl.c.o: libartl/artl.c
CMakeFiles/standing_controlV2.dir/libartl/artl.c.o: CMakeFiles/standing_controlV2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/standing_controlV2.dir/libartl/artl.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/standing_controlV2.dir/libartl/artl.c.o -MF CMakeFiles/standing_controlV2.dir/libartl/artl.c.o.d -o CMakeFiles/standing_controlV2.dir/libartl/artl.c.o -c /home/orl/Tianze_WS/Test_Control/libartl/artl.c

CMakeFiles/standing_controlV2.dir/libartl/artl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/standing_controlV2.dir/libartl/artl.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/orl/Tianze_WS/Test_Control/libartl/artl.c > CMakeFiles/standing_controlV2.dir/libartl/artl.c.i

CMakeFiles/standing_controlV2.dir/libartl/artl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/standing_controlV2.dir/libartl/artl.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/orl/Tianze_WS/Test_Control/libartl/artl.c -o CMakeFiles/standing_controlV2.dir/libartl/artl.c.s

CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.o: CMakeFiles/standing_controlV2.dir/flags.make
CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.o: libartl/artl_internal.c
CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.o: CMakeFiles/standing_controlV2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.o -MF CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.o.d -o CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.o -c /home/orl/Tianze_WS/Test_Control/libartl/artl_internal.c

CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/orl/Tianze_WS/Test_Control/libartl/artl_internal.c > CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.i

CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/orl/Tianze_WS/Test_Control/libartl/artl_internal.c -o CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.s

CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.o: CMakeFiles/standing_controlV2.dir/flags.make
CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.o: libartl/crc32c.c
CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.o: CMakeFiles/standing_controlV2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.o -MF CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.o.d -o CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.o -c /home/orl/Tianze_WS/Test_Control/libartl/crc32c.c

CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/orl/Tianze_WS/Test_Control/libartl/crc32c.c > CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.i

CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/orl/Tianze_WS/Test_Control/libartl/crc32c.c -o CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.s

CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.o: CMakeFiles/standing_controlV2.dir/flags.make
CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.o: libartl/zstd/zstd.c
CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.o: CMakeFiles/standing_controlV2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.o -MF CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.o.d -o CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.o -c /home/orl/Tianze_WS/Test_Control/libartl/zstd/zstd.c

CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/orl/Tianze_WS/Test_Control/libartl/zstd/zstd.c > CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.i

CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/orl/Tianze_WS/Test_Control/libartl/zstd/zstd.c -o CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.s

CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.o: CMakeFiles/standing_controlV2.dir/flags.make
CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.o: src/kin_left_arm.cpp
CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.o: CMakeFiles/standing_controlV2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.o -MF CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.o.d -o CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.o -c /home/orl/Tianze_WS/Test_Control/src/kin_left_arm.cpp

CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/orl/Tianze_WS/Test_Control/src/kin_left_arm.cpp > CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.i

CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/orl/Tianze_WS/Test_Control/src/kin_left_arm.cpp -o CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.s

CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.o: CMakeFiles/standing_controlV2.dir/flags.make
CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.o: src/kin_right_arm.cpp
CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.o: CMakeFiles/standing_controlV2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.o -MF CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.o.d -o CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.o -c /home/orl/Tianze_WS/Test_Control/src/kin_right_arm.cpp

CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/orl/Tianze_WS/Test_Control/src/kin_right_arm.cpp > CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.i

CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/orl/Tianze_WS/Test_Control/src/kin_right_arm.cpp -o CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.s

# Object files for target standing_controlV2
standing_controlV2_OBJECTS = \
"CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.o" \
"CMakeFiles/standing_controlV2.dir/lowlevelapi.c.o" \
"CMakeFiles/standing_controlV2.dir/libartl/artl.c.o" \
"CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.o" \
"CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.o" \
"CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.o" \
"CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.o" \
"CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.o"

# External object files for target standing_controlV2
standing_controlV2_EXTERNAL_OBJECTS =

standing_controlV2: CMakeFiles/standing_controlV2.dir/standing_controlV2.cpp.o
standing_controlV2: CMakeFiles/standing_controlV2.dir/lowlevelapi.c.o
standing_controlV2: CMakeFiles/standing_controlV2.dir/libartl/artl.c.o
standing_controlV2: CMakeFiles/standing_controlV2.dir/libartl/artl_internal.c.o
standing_controlV2: CMakeFiles/standing_controlV2.dir/libartl/crc32c.c.o
standing_controlV2: CMakeFiles/standing_controlV2.dir/libartl/zstd/zstd.c.o
standing_controlV2: CMakeFiles/standing_controlV2.dir/src/kin_left_arm.cpp.o
standing_controlV2: CMakeFiles/standing_controlV2.dir/src/kin_right_arm.cpp.o
standing_controlV2: CMakeFiles/standing_controlV2.dir/build.make
standing_controlV2: /home/orl/anaconda3/lib/libOsqpEigen.so.0.8.1
standing_controlV2: include/AnalyticalKinematicsDynamics/libkinematics_dynamics_lib.a
standing_controlV2: libdigit_safety_lib.a
standing_controlV2: /home/orl/anaconda3/lib/libosqp.so
standing_controlV2: CMakeFiles/standing_controlV2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX executable standing_controlV2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/standing_controlV2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/standing_controlV2.dir/build: standing_controlV2
.PHONY : CMakeFiles/standing_controlV2.dir/build

CMakeFiles/standing_controlV2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/standing_controlV2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/standing_controlV2.dir/clean

CMakeFiles/standing_controlV2.dir/depend:
	cd /home/orl/Tianze_WS/Test_Control && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/orl/Tianze_WS/Test_Control /home/orl/Tianze_WS/Test_Control /home/orl/Tianze_WS/Test_Control /home/orl/Tianze_WS/Test_Control /home/orl/Tianze_WS/Test_Control/CMakeFiles/standing_controlV2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/standing_controlV2.dir/depend

