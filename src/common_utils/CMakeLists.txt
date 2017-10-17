cmake_minimum_required(VERSION 3.5)
project(spartan_common_utils)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

set(HEADERS
	math_utils.h
	system_utils.h
	vtk_utils.h)

install(FILES ${HEADERS}
	DESTINATION include/common_utils)
