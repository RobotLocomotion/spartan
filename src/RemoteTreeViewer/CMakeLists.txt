cmake_minimum_required(VERSION 3.5)
project(RemoteTreeViewer)
set(CMAKE_CXX_STANDARD 11)

# special care is needed when targeting Drake since the drake-config.cmake is broken
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake/modules")
find_package(Drake MODULE REQUIRED)
find_package(Eigen REQUIRED)

  include_directories(SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${DRAKE_INCLUDE_DIRS}
  )
find_package(yaml-cpp)
find_package(lcm)

add_library(RemoteTreeViewerWrapper SHARED RemoteTreeViewerWrapper.cpp)
# target_link_libraries(RemoteTreeViewerWrapper lcm drakeRBM drakeJoints drakeShapes drakeCommon z pthread)
target_link_libraries(RemoteTreeViewerWrapper
	${DRAKE_LIBRARIES}
	lcm
	z
	pthread)
install(TARGETS RemoteTreeViewerWrapper DESTINATION lib)
install(FILES RemoteTreeViewerWrapper.hpp DESTINATION include)

add_executable(testRemoteTreeViewerWrapper testRemoteTreeViewerWrapper.cpp)
target_link_libraries(testRemoteTreeViewerWrapper
	lcm
	${DRAKE_LIBRARIES}
	yaml-cpp
	z
	pthread
	RemoteTreeViewerWrapper)
install(TARGETS testRemoteTreeViewerWrapper DESTINATION bin)
