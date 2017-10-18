cmake_minimum_required(VERSION 3.5)
project(iiwa_rlg_sim_backend)
set(CMAKE_CXX_STANDARD 11)

find_package(drake REQUIRED)
find_package(gflags REQUIRED)

# Hah I'm circumventing catkin
find_package(roscpp REQUIRED)
find_package(image_transport REQUIRED)

set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIR}) 
message("Eigen3 include dir" ${EIGEN3_INCLUDE_DIR})

link_directories(${CMAKE_INSTALL_PREFIX}/lib)

find_package(bot2-core-lcmtypes REQUIRED)
add_library(image_to_images_t image_driver/image_to_images_t.cc image_driver/image_to_images_t.h)
target_link_libraries(image_to_images_t
    z
    pthread
    ${bot2-core-lcmtypes_LIBRARIES}
    ${drake_LIBRARIES}
    ${gflags_LIBRARIES})

add_library(rgbd_to_ros_pointcloud2_driver image_driver/rgbd_to_ros_pointcloud2_driver.cc image_driver/rgbd_to_ros_pointcloud2_driver.h)
target_link_libraries(rgbd_to_ros_pointcloud2_driver
    z
    pthread
    ${roscpp_LIBRARIES}
    ${image_transport_LIBRARIES}
    ${drake_LIBRARIES}
    ${gflags_LIBRARIES})


add_library(iiwa_common iiwa_common.cc iiwa_common.h)
target_link_libraries(iiwa_common
    z
    pthread
    ${drake_LIBRARIES}
    ${gflags_LIBRARIES})

add_library(iiwa_lcm iiwa_lcm.cc iiwa_lcm.h)
target_link_libraries(iiwa_lcm
    z
    pthread
    ${drake_LIBRARIES}
    ${gflags_LIBRARIES})

add_library(iiwa_oracular_state_estimator oracular_state_estimator.cc oracular_state_estimator.h)
target_link_libraries(iiwa_oracular_state_estimator
    z
    pthread
    ${drake_LIBRARIES}
    ${gflags_LIBRARIES})

add_library(iiwa_wsg_diagram_factory iiwa_world/iiwa_wsg_diagram_factory.cc iiwa_world/iiwa_wsg_diagram_factory.h)
target_link_libraries(iiwa_wsg_diagram_factory
    z
    pthread
    iiwa_common
    iiwa_oracular_state_estimator
    ${drake_LIBRARIES}
    ${gflags_LIBRARIES})

add_executable(iiwa_rlg_simulation iiwa_rlg_simulation.cc)

target_link_libraries(iiwa_rlg_simulation
    z
    pthread
    ${bot2-core-lcmtypes_LIBRARIES}
    image_to_images_t
    rgbd_to_ros_pointcloud2_driver
    iiwa_common
    iiwa_lcm
    iiwa_wsg_diagram_factory
    ${roscpp_LIBRARIES}
    ${drake_LIBRARIES}
    ${gflags_LIBRARIES}
    RemoteTreeViewerWrapper)

install(TARGETS iiwa_rlg_simulation DESTINATION bin)
install(TARGETS image_to_images_t iiwa_common iiwa_lcm iiwa_oracular_state_estimator rgbd_to_ros_pointcloud2_driver iiwa_wsg_diagram_factory DESTINATION lib)