cmake_minimum_required(VERSION 3.5)
project(spartan_common_utils)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

add_library(eigen_nd_array eigen_nd_array.cc eigen_nd_array.h)
target_link_libraries(eigen_nd_array
	${Eigen3_LIBRARIES})

add_library(eigen_histogram eigen_histogram.cc eigen_histogram.h)
target_link_libraries(eigen_histogram
	eigen_nd_array
	${Eigen3_LIBRARIES})

add_executable(test_eigen_histogram test/test_eigen_histogram.cc)
target_link_libraries(test_eigen_histogram
	eigen_nd_array
	eigen_histogram
	${Eigen3_LIBRARIES})

set(HEADERS
	cv_utils.h
	eigen_nd_array.h
	eigen_histogram.h
	math_utils.h
	system_utils.h
	pcl_utils.h
	pcl_vtk_utils.h
	vtk_utils.h)

install(FILES ${HEADERS}
	DESTINATION include/common_utils)

install(TARGETS eigen_nd_array eigen_histogram DESTINATION lib)
install(TARGETS test_eigen_histogram DESTINATION bin)
