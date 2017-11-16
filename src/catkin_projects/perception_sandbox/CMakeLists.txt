cmake_minimum_required(VERSION 2.8.3)
project(perception_sandbox)
set(CMAKE_CXX_STANDARD 11)

find_package(gflags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	sensor_msgs
	cv_bridge
	roscpp
	image_transport
   pcl_conversions
   pcl_ros
)

catkin_package(
# INCLUDE_DIRS
# CATKIN_DEPENDS message_runtime
#  LIBRARIES perception_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


###########
## Build ##
###########


add_executable(opencv_view_node src/opencv_view_node.cpp)
target_include_directories(opencv_view_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(opencv_view_node
   ${catkin_LIBRARIES}
)

# There must be a better way... gotta figure out
# how to make find_package work for common utils
include_directories(${CMAKE_INSTALL_PREFIX}/include)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

# While this library relies on PCL and OpenCV, they are brought in
# by catkin_LIBRARIES. I think this finds the ROS-supplied (system installed)
# OpenCV and PCL. In the current configuration, these are close enough that
# this doesn't pose an issue. If you want to involve Drake in a node
# like this down the road, see
# https://github.com/RobotLocomotion/drake-shambhala/issues/38
# for some ideas of how to resolve the Eigen compatibility issues you'll have.
add_executable(pointcloud2_capture_node src/pointcloud2_capture_node.cpp)
target_include_directories(pointcloud2_capture_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(pointcloud2_capture_node
   ${catkin_LIBRARIES}
   ${gflags_LIBRARIES}
)


#############
## Install ##
#############

#catkin_install_python(PROGRAMS
#   scripts/rgbd_frame_grabber.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS opencv_view_node pointcloud2_capture_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#   launch/realsense_camera_view.launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)