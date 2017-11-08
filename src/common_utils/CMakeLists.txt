cmake_minimum_required(VERSION 3.5)
project(spartan_common_utils)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_library(CVUTILS_LIBRARY cv-utils DOC "The cv-utils library")
find_path(CVUTILS_INCLUDE_DIR image_utils/jpeg.h PATH_SUFFIXES cv-utils DOC "Path to the cv-utils include directory")
set(CVUTILS_INCLUDE_DIRS ${CVUTILS_INCLUDE_DIR})
set(CVUTILS_LIBRARIES ${CVUTILS_LIBRARY})
include_directories(${CVUTILS_INCLUDE_DIRS})

find_package(drake REQUIRED)

set(HEADERS
	math_utils.h
	system_utils.h
	vtk_utils.h
	rgbd_camera_driver.h)

install(FILES ${HEADERS}
	DESTINATION include/common_utils)

add_library(rgbd_camera_driver rgbd_camera_driver.cc rgbd_camera_driver.h)
target_link_libraries(rgbd_camera_driver
    ${drake_LIBRARIES}
	${OpenCV_LIBS}
    ${CVUTILS_LIBRARIES})
install(TARGETS rgbd_camera_driver DESTINATION lib)