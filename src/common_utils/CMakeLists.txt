cmake_minimum_required(VERSION 3.5)
project(spartan_common_utils)
set(CMAKE_CXX_STANDARD 11)

set(HEADERS
	cv_utils.h
	math_utils.h
	system_utils.h
	pcl_utils.h
	vtk_utils.h)

install(FILES ${HEADERS}
	DESTINATION include/common_utils)
