cmake_minimum_required(VERSION 2.8.3)
project(ur_modern_driver)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(lcm)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake/modules")

# find packages that don't work with normal find_package
find_package(Drake MODULE REQUIRED)
find_package(Eigen REQUIRED)

  include_directories(SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${DRAKE_INCLUDE_DIRS}
    include
    lcmtypes
  )

###########
## Build ##
###########

# Hardware Interface
#add_library(ur_hardware_interface src/ur_hardware_interface.cpp)
#target_link_libraries(ur_hardware_interface
#    ${DRAKE_LIBRARIES}
#    lcm
#)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(ur_modern_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
set(${PROJECT_NAME}_SOURCES
    src/ur_ros_wrapper.cpp
    src/ur_driver.cpp
    src/ur_realtime_communication.cpp
    src/ur_communication.cpp
    src/robot_state.cpp
    src/robot_state_RT.cpp
    src/do_output.cpp)
add_executable(ur_driver ${${PROJECT_NAME}_SOURCES})

## Specify libraries to link a library or executable target against
target_link_libraries(ur_driver
  ${EIGEN_INCLUDE_DIRS}
  lcm
  pthread
 )

#############
## Install ##
#############

install(TARGETS ur_driver 
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
