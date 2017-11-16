cmake_minimum_required(VERSION 2.8.3)
project(perception_sandbox)
set(CMAKE_CXX_STANDARD 11)

find_package(drake CONFIG REQUIRED)
find_package(Eigen3 3.3.3 REQUIRED)
find_package(PCL CONFIG REQUIRED)

find_package(OpenCV REQUIRED)
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

add_executable(pointcloud2_capture_node src/pointcloud2_capture_node.cpp)
target_include_directories(pointcloud2_capture_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(pointcloud2_capture_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${gflags_LIBRARIES}
)

add_executable(object_fitting_node src/object_fitting_node.cpp)
# This is nasty: Drake wants to provide Eigen 3.3.3, while ROS prefers the system-installed
# (and older) version. Drake will fail to compile if it sees the system-installed
# version. However, some part of the CMake configuration winds up specifying the Eigen
# include path as a SYSTEM include path, so we need to preemptively specify
# OUR Eigen as a SYSTEM include path as well, and then declare the catkin include paths
# as ALSO SYSTEM include paths. (-isystem vs -I, where I think -I takes precedence over
# -isystem where both are present and valid).
target_include_directories(object_fitting_node SYSTEM PUBLIC ${EIGEN_INCLUDE_DIRS})
target_include_directories(object_fitting_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(object_fitting_node
   drake::drake
   ${Eigen3_LIBRARIES}
   RemoteTreeViewerWrapper
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${gflags_LIBRARIES}
)


#############
## Install ##
#############

#catkin_install_python(PROGRAMS
#   scripts/rgbd_frame_grabber.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS opencv_view_node pointcloud2_capture_node object_fitting_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   launch/realsense_camera_view.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)