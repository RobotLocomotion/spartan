cmake_minimum_required(VERSION 2.8.3)
project(drake_simulation)
set(CMAKE_CXX_STANDARD 11)

find_package(drake REQUIRED)
# Drake's up-to-date Eigen
find_package(Eigen3 3.3.3 REQUIRED)
find_package(GFlags MODULE REQUIRED COMPONENTS shared)
find_package(yaml-cpp REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    sensor_msgs
    roscpp
    image_transport
    drake_ros_systems
    wsg50_msgs
)

catkin_package(
# INCLUDE_DIRS
# CATKIN_DEPENDS message_runtime
#  LIBRARIES perception_msgs
  CATKIN_DEPENDS roscpp drake_ros_systems wsg50_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

# This is wrong, but drake_ros_systems_INCLUDE_DIRS is build/install/include,
# which doesn't get populated till the install step.
include_directories(${drake_ros_systems_SOURCE_DIR}/include)

# There must be a better way... gotta figure out
# how to make find_package work for common utils
include_directories(${CMAKE_INSTALL_PREFIX}/include)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)


#add_library(time_stepping_rigid_body_plant
#    src/time_stepping_rigid_body_plant/time_stepping_rigid_body_plant.cc
#    src/time_stepping_rigid_body_plant/time_stepping_rigid_body_plant.h)
#target_link_libraries(time_stepping_rigid_body_plant
#    ${drake_LIBRARIES})


add_executable(drake_passive_simulation_from_config src/drake_passive_simulation_from_config.cc)
target_link_libraries(drake_passive_simulation_from_config
    ${catkin_LIBRARIES}
    ${drake_LIBRARIES}
    gflags_shared
    yaml-cpp
    ${EIGEN3_LIBRARIES}
)

add_library(rgbd_to_ros_pointcloud2_driver
    src/iiwa_rlg_simulation/image_driver/rgbd_to_ros_pointcloud2_driver.cc
    src/iiwa_rlg_simulation/image_driver/rgbd_to_ros_pointcloud2_driver.h)
target_link_libraries(rgbd_to_ros_pointcloud2_driver
    ${catkin_LIBRARIES}
    ${drake_LIBRARIES})

add_library(image_to_images_t
    src/iiwa_rlg_simulation/image_driver/image_to_images_t.cc
    src/iiwa_rlg_simulation/image_driver/image_to_images_t.h)
target_link_libraries(image_to_images_t
    ${catkin_LIBRARIES}
    ${drake_LIBRARIES})

add_library(schunk_wsg_ros
    src/iiwa_rlg_simulation/schunk_wsg_ros/schunk_wsg_ros.cc
    src/iiwa_rlg_simulation/schunk_wsg_ros/schunk_wsg_ros.h)
add_dependencies(schunk_wsg_ros ${catkin_EXPORTED_TARGETS})
target_link_libraries(schunk_wsg_ros
    ${catkin_LIBRARIES}
    ${drake_LIBRARIES})
add_library(schunk_wsg_ros_controller
    src/iiwa_rlg_simulation/schunk_wsg_ros/schunk_wsg_ros_controller.cc
    src/iiwa_rlg_simulation/schunk_wsg_ros/schunk_wsg_ros_controller.h)
add_dependencies(schunk_wsg_ros_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(schunk_wsg_ros_controller
    ${catkin_LIBRARIES}
    ${drake_LIBRARIES})


add_library(iiwa_common
    src/iiwa_rlg_simulation/iiwa_common.cc 
    src/iiwa_rlg_simulation/iiwa_common.h)
target_link_libraries(iiwa_common
    ${drake_LIBRARIES})

add_library(iiwa_lcm
    src/iiwa_rlg_simulation/iiwa_lcm.cc 
    src/iiwa_rlg_simulation/iiwa_lcm.h)
target_link_libraries(iiwa_lcm
    ${drake_LIBRARIES})

add_library(iiwa_oracular_state_estimator
    src/iiwa_rlg_simulation/oracular_state_estimator.cc 
    src/iiwa_rlg_simulation/oracular_state_estimator.h)
target_link_libraries(iiwa_oracular_state_estimator
    ${drake_LIBRARIES})

add_library(iiwa_wsg_diagram_factory
    src/iiwa_rlg_simulation/iiwa_world/iiwa_wsg_diagram_factory.cc 
    src/iiwa_rlg_simulation/iiwa_world/iiwa_wsg_diagram_factory.h)
target_link_libraries(iiwa_wsg_diagram_factory
    iiwa_common
    iiwa_oracular_state_estimator
    ${drake_LIBRARIES})

add_executable(iiwa_rlg_simulation 
    src/iiwa_rlg_simulation/iiwa_rlg_simulation.cc)
add_dependencies(iiwa_rlg_simulation ${catkin_EXPORTED_TARGETS})
target_link_libraries(iiwa_rlg_simulation
    ${catkin_LIBRARIES}
    ${bot2-core-lcmtypes_LIBRARIES}
    yaml-cpp
    image_to_images_t
    rgbd_to_ros_pointcloud2_driver
    iiwa_common
    iiwa_lcm
    iiwa_wsg_diagram_factory
    iiwa_oracular_state_estimator
    schunk_wsg_ros
    schunk_wsg_ros_controller
    gflags_shared
    ${drake_LIBRARIES}
    RemoteTreeViewerWrapper)


#############
## Install ##
#############

install(PROGRAMS
    scripts/pybullet_passive_simulation_from_config.py
    scripts/pybullet_iiwa_rlg_simulation.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS drake_passive_simulation_from_config
   iiwa_rlg_simulation
   image_to_images_t
   rgbd_to_ros_pointcloud2_driver
   iiwa_common
   iiwa_lcm
   iiwa_wsg_diagram_factory
   iiwa_oracular_state_estimator
   schunk_wsg_ros
   schunk_wsg_ros_controller
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)