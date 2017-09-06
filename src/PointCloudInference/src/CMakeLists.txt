cmake_minimum_required(VERSION 2.6.0)
cmake_policy(SET CMP0045 OLD)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -std=gnu++14 -fext-numeric-literals")

include_directories(${CMAKE_INSTALL_PREFIX})
include_directories(${CMAKE_INSTALL_PREFIX}/include)
include_directories(${CMAKE_INSTALL_PREFIX}/include/lcmtypes)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIR}) 
message("Eigen3 include dir" ${EIGEN3_INCLUDE_DIR})

find_package(drake)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_library(eigen_nd_array eigen_nd_array.cc eigen_nd_array.h)
target_link_libraries(eigen_nd_array)
install(TARGETS eigen_nd_array DESTINATION lib)

add_library(voxel_distance_field voxel_distance_field.cc voxel_distance_field.h)
target_link_libraries(voxel_distance_field eigen_nd_array)
install(TARGETS voxel_distance_field DESTINATION lib)

add_executable(test_voxel_distance_field test_voxel_distance_field.cc)
target_link_libraries(test_voxel_distance_field voxel_distance_field eigen_nd_array RemoteTreeViewerWrapper drakeLcm)
install(TARGETS test_voxel_distance_field DESTINATION bin)

add_executable(convert_pointcloud_to_occupancy_and_sdf convert_pointcloud_to_occupancy_and_sdf.cc)
target_link_libraries(convert_pointcloud_to_occupancy_and_sdf eigen_nd_array voxel_distance_field lcm drakeLcm RemoteTreeViewerWrapper yaml-cpp ${VTK_LIBRARIES} ${PCL_LIBRARIES} z pthread)
install(TARGETS convert_pointcloud_to_occupancy_and_sdf DESTINATION bin)
