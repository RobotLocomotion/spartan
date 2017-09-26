cmake_minimum_required(VERSION 3.5)
project(ReachabilityAnalyzer)
set(CMAKE_CXX_STANDARD 11)

find_package(drake REQUIRED)
find_package(gflags REQUIRED)

set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIR}) 
message("Eigen3 include dir" ${EIGEN3_INCLUDE_DIR})

link_directories(${CMAKE_INSTALL_PREFIX}/lib)

add_executable(reachability_analyzer reachability_analyzer.cpp)
target_link_libraries(reachability_analyzer
    z
    pthread
    ${drake_LIBRARIES}
    ${gflags_LIBRARIES}
    RemoteTreeViewerWrapper)

install(TARGETS reachability_analyzer DESTINATION bin)
