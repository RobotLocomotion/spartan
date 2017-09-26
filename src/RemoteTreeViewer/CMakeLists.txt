cmake_minimum_required(VERSION 3.5)
project(RemoteTreeViewer)
set(CMAKE_CXX_STANDARD 11)

find_package(drake REQUIRED)

add_library(RemoteTreeViewerWrapper SHARED RemoteTreeViewerWrapper.cpp)
target_link_libraries(RemoteTreeViewerWrapper
    drake::drake
    z
    pthread)
install(TARGETS RemoteTreeViewerWrapper DESTINATION lib)
install(FILES RemoteTreeViewerWrapper.hpp DESTINATION include)

add_executable(testRemoteTreeViewerWrapper testRemoteTreeViewerWrapper.cpp)
target_link_libraries(testRemoteTreeViewerWrapper
    z
    pthread
    RemoteTreeViewerWrapper)
install(TARGETS testRemoteTreeViewerWrapper DESTINATION bin)
