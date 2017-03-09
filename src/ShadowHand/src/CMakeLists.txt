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

add_executable(run_shadow_hand_sim run_shadow_hand_sim.cpp)
target_link_libraries(run_shadow_hand_sim lcm RemoteTreeViewerWrapper drakeMultibodyParsers drakeTrajectories drakeLCMUtil drakeRBM drakeShapes drakeJoints drakeCommon drakeLcm drakeRigidBodyPlant drake_lcmtypes yaml-cpp z pthread)
install(TARGETS run_shadow_hand_sim DESTINATION bin)
