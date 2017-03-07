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

find_package(drake REQUIRED)

add_library(point_cloud_generator point_cloud_generator.cpp point_cloud_generator.hpp)
target_link_libraries(point_cloud_generator lcm RemoteTreeViewerWrapper drakeMultibodyParsers drakeRBM drakeOptimization drakeShapes drakeJoints drakeCommon drakeLcm drakeRigidBodyPlant drake_lcmtypes yaml-cpp z pthread)
install(TARGETS point_cloud_generator DESTINATION lib)

add_executable(run_point_cloud_generator run_point_cloud_generator.cpp)
target_link_libraries(run_point_cloud_generator lcm RemoteTreeViewerWrapper point_cloud_generator drakeMultibodyParsers drakeRBM drakeShapes drakeJoints drakeCommon drakeLcm drakeRigidBodyPlant drake_lcmtypes yaml-cpp z pthread)
install(TARGETS run_point_cloud_generator DESTINATION bin)