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
include CMakeFiles/cmake_hello.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/cmake_hello.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/cmake_hello.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cmake_hello.dir/flags.make

CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.o: CMakeFiles/cmake_hello.dir/flags.make
CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.o: lowlevelapi_example.cpp
CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.o: CMakeFiles/cmake_hello.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.o -MF CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.o.d -o CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.o -c /home/orl/Tianze_WS/Test_Control/lowlevelapi_example.cpp

CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/orl/Tianze_WS/Test_Control/lowlevelapi_example.cpp > CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.i

CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/orl/Tianze_WS/Test_Control/lowlevelapi_example.cpp -o CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.s

CMakeFiles/cmake_hello.dir/lowlevelapi.c.o: CMakeFiles/cmake_hello.dir/flags.make
CMakeFiles/cmake_hello.dir/lowlevelapi.c.o: lowlevelapi.c
CMakeFiles/cmake_hello.dir/lowlevelapi.c.o: CMakeFiles/cmake_hello.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/cmake_hello.dir/lowlevelapi.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/cmake_hello.dir/lowlevelapi.c.o -MF CMakeFiles/cmake_hello.dir/lowlevelapi.c.o.d -o CMakeFiles/cmake_hello.dir/lowlevelapi.c.o -c /home/orl/Tianze_WS/Test_Control/lowlevelapi.c

CMakeFiles/cmake_hello.dir/lowlevelapi.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/cmake_hello.dir/lowlevelapi.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/orl/Tianze_WS/Test_Control/lowlevelapi.c > CMakeFiles/cmake_hello.dir/lowlevelapi.c.i

CMakeFiles/cmake_hello.dir/lowlevelapi.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/cmake_hello.dir/lowlevelapi.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/orl/Tianze_WS/Test_Control/lowlevelapi.c -o CMakeFiles/cmake_hello.dir/lowlevelapi.c.s

CMakeFiles/cmake_hello.dir/libartl/artl.c.o: CMakeFiles/cmake_hello.dir/flags.make
CMakeFiles/cmake_hello.dir/libartl/artl.c.o: libartl/artl.c
CMakeFiles/cmake_hello.dir/libartl/artl.c.o: CMakeFiles/cmake_hello.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/cmake_hello.dir/libartl/artl.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/cmake_hello.dir/libartl/artl.c.o -MF CMakeFiles/cmake_hello.dir/libartl/artl.c.o.d -o CMakeFiles/cmake_hello.dir/libartl/artl.c.o -c /home/orl/Tianze_WS/Test_Control/libartl/artl.c

CMakeFiles/cmake_hello.dir/libartl/artl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/cmake_hello.dir/libartl/artl.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/orl/Tianze_WS/Test_Control/libartl/artl.c > CMakeFiles/cmake_hello.dir/libartl/artl.c.i

CMakeFiles/cmake_hello.dir/libartl/artl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/cmake_hello.dir/libartl/artl.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/orl/Tianze_WS/Test_Control/libartl/artl.c -o CMakeFiles/cmake_hello.dir/libartl/artl.c.s

CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.o: CMakeFiles/cmake_hello.dir/flags.make
CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.o: libartl/artl_internal.c
CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.o: CMakeFiles/cmake_hello.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.o -MF CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.o.d -o CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.o -c /home/orl/Tianze_WS/Test_Control/libartl/artl_internal.c

CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/orl/Tianze_WS/Test_Control/libartl/artl_internal.c > CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.i

CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/orl/Tianze_WS/Test_Control/libartl/artl_internal.c -o CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.s

CMakeFiles/cmake_hello.dir/libartl/crc32c.c.o: CMakeFiles/cmake_hello.dir/flags.make
CMakeFiles/cmake_hello.dir/libartl/crc32c.c.o: libartl/crc32c.c
CMakeFiles/cmake_hello.dir/libartl/crc32c.c.o: CMakeFiles/cmake_hello.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/cmake_hello.dir/libartl/crc32c.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/cmake_hello.dir/libartl/crc32c.c.o -MF CMakeFiles/cmake_hello.dir/libartl/crc32c.c.o.d -o CMakeFiles/cmake_hello.dir/libartl/crc32c.c.o -c /home/orl/Tianze_WS/Test_Control/libartl/crc32c.c

CMakeFiles/cmake_hello.dir/libartl/crc32c.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/cmake_hello.dir/libartl/crc32c.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/orl/Tianze_WS/Test_Control/libartl/crc32c.c > CMakeFiles/cmake_hello.dir/libartl/crc32c.c.i

CMakeFiles/cmake_hello.dir/libartl/crc32c.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/cmake_hello.dir/libartl/crc32c.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/orl/Tianze_WS/Test_Control/libartl/crc32c.c -o CMakeFiles/cmake_hello.dir/libartl/crc32c.c.s

CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.o: CMakeFiles/cmake_hello.dir/flags.make
CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.o: libartl/zstd/zstd.c
CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.o: CMakeFiles/cmake_hello.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.o -MF CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.o.d -o CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.o -c /home/orl/Tianze_WS/Test_Control/libartl/zstd/zstd.c

CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/orl/Tianze_WS/Test_Control/libartl/zstd/zstd.c > CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.i

CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/orl/Tianze_WS/Test_Control/libartl/zstd/zstd.c -o CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.s

# Object files for target cmake_hello
cmake_hello_OBJECTS = \
"CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.o" \
"CMakeFiles/cmake_hello.dir/lowlevelapi.c.o" \
"CMakeFiles/cmake_hello.dir/libartl/artl.c.o" \
"CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.o" \
"CMakeFiles/cmake_hello.dir/libartl/crc32c.c.o" \
"CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.o"

# External object files for target cmake_hello
cmake_hello_EXTERNAL_OBJECTS =

cmake_hello: CMakeFiles/cmake_hello.dir/lowlevelapi_example.cpp.o
cmake_hello: CMakeFiles/cmake_hello.dir/lowlevelapi.c.o
cmake_hello: CMakeFiles/cmake_hello.dir/libartl/artl.c.o
cmake_hello: CMakeFiles/cmake_hello.dir/libartl/artl_internal.c.o
cmake_hello: CMakeFiles/cmake_hello.dir/libartl/crc32c.c.o
cmake_hello: CMakeFiles/cmake_hello.dir/libartl/zstd/zstd.c.o
cmake_hello: CMakeFiles/cmake_hello.dir/build.make
cmake_hello: /home/orl/anaconda3/lib/libOsqpEigen.so.0.8.1
cmake_hello: include/AnalyticalKinematicsDynamics/libkinematics_dynamics_lib.a
cmake_hello: /home/orl/anaconda3/lib/libosqp.so
cmake_hello: CMakeFiles/cmake_hello.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/orl/Tianze_WS/Test_Control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable cmake_hello"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cmake_hello.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cmake_hello.dir/build: cmake_hello
.PHONY : CMakeFiles/cmake_hello.dir/build

CMakeFiles/cmake_hello.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cmake_hello.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cmake_hello.dir/clean

CMakeFiles/cmake_hello.dir/depend:
	cd /home/orl/Tianze_WS/Test_Control && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/orl/Tianze_WS/Test_Control /home/orl/Tianze_WS/Test_Control /home/orl/Tianze_WS/Test_Control /home/orl/Tianze_WS/Test_Control /home/orl/Tianze_WS/Test_Control/CMakeFiles/cmake_hello.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cmake_hello.dir/depend

