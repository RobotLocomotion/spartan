cmake_minimum_required(VERSION 3.5)
project(dishrack_simulation)
set(CMAKE_CXX_STANDARD 11)

find_package(drake REQUIRED)
find_package(gflags REQUIRED)

find_package(Eigen3)
include_directories(${Eigen3_INCLUDE_DIR}) 

link_directories(${CMAKE_INSTALL_PREFIX}/lib)

add_executable(dishrack_simulation dishrack_simulation.cc)

target_link_libraries(dishrack_simulation
    z
    pthread
    ${bot2-core-lcmtypes_LIBRARIES}
    ${drake_LIBRARIES}
    ${gflags_LIBRARIES}
    RemoteTreeViewerWrapper)

install(TARGETS dishrack_simulation DESTINATION bin)