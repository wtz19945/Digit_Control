cmake_minimum_required(VERSION 3.14.1)
project(CMakeHello)

set(Home_Dir ${CMAKE_CURRENT_SOURCE_DIR}) # change this to your directory 
include_directories(/libartl
  include/GeometricKinematics
  include/AnalyticalKinematicsDynamics
  include/Eigen/
  ${Home_Dir}/include/Eigen
  ${Home_Dir}/include
  ${Home_Dir}/include/cpptoml)
add_subdirectory(include/GeometricKinematics)
add_subdirectory(include/AnalyticalKinematicsDynamics)

set(Digit_Source 
	lowlevelapi.c
	lowlevelapi.h
	libartl/artl.c
	libartl/artl.h
	libartl/artl_internal.c
	libartl/artl_internal.h
	libartl/crc32c.c
	libartl/crc32c.h
	libartl/zstd/zstd.c
	libartl/zstd/zstd.h
        src/kin_left_arm.cpp
	src/kin_right_arm.cpp)

find_package(OsqpEigen REQUIRED)
set(SOURCES lowlevelapi_example.cpp
${Digit_Source})

set(standing_SOURCES standing_control.cpp
${Digit_Source})

set(standing_SOURCESV2 standing_controlV2.cpp
${Digit_Source})

add_executable(cmake_hello ${SOURCES})
target_link_libraries(cmake_hello OsqpEigen::OsqpEigen kinematics_dynamics_lib)

add_executable(standing_control ${standing_SOURCES})
target_link_libraries(standing_control OsqpEigen::OsqpEigen kinematics_dynamics_lib)

add_executable(standing_controlV2 ${standing_SOURCESV2})
target_link_libraries(standing_controlV2 OsqpEigen::OsqpEigen kinematics_dynamics_lib)
