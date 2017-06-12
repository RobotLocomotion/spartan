cmake_minimum_required(VERSION 2.6.0)
cmake_policy(SET CMP0045 OLD)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${CMAKE_INSTALL_PREFIX})
include_directories(${CMAKE_INSTALL_PREFIX}/include)
include_directories(${CMAKE_INSTALL_PREFIX}/include/lcmtypes)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIR}) 
message("Eigen3 include dir" ${EIGEN3_INCLUDE_DIR})


list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake/modules")
find_package(Drake MODULE REQUIRED)
find_package(Eigen REQUIRED)

  include_directories(SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${DRAKE_INCLUDE_DIRS}
  )

find_package(lcm)
find_package(yaml-cpp)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(point_cloud_generator point_cloud_generator.cpp point_cloud_generator.hpp)
target_link_libraries(point_cloud_generator
	lcm
	RemoteTreeViewerWrapper
	${DRAKE_LIBRARIES}
	yaml-cpp
	${PCL_LIBRARIES}
	z
	pthread)
install(TARGETS point_cloud_generator DESTINATION lib)

add_executable(run_point_cloud_generator run_point_cloud_generator.cpp)
target_link_libraries(run_point_cloud_generator
	point_cloud_generator
	lcm
	RemoteTreeViewerWrapper
	${DRAKE_LIBRARIES}
	yaml-cpp
	${PCL_LIBRARIES}
	z
	pthread)
install(TARGETS run_point_cloud_generator DESTINATION bin)

add_executable(run_bogo_object_detector run_bogo_object_detector.cpp)
target_link_libraries(run_bogo_object_detector
	point_cloud_generator
	lcm
	RemoteTreeViewerWrapper
	${DRAKE_LIBRARIES}
	yaml-cpp
	${PCL_LIBRARIES}
	z
	pthread)
install(TARGETS run_bogo_object_detector DESTINATION bin)