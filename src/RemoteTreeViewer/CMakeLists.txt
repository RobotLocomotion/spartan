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


set(pybind11_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake/pybind11")
set(PYTHON_EXECUTABLE "python2.7")
find_package(pybind11 REQUIRED)

pybind11_add_module(RemoteTreeViewerWrapper_pybind MODULE RemoteTreeViewerWrapper_pybind.cpp)
target_link_libraries(RemoteTreeViewerWrapper_pybind PRIVATE RemoteTreeViewerWrapper drake::drake z pthread)
set_target_properties(RemoteTreeViewerWrapper_pybind PROPERTIES PREFIX "" SUFFIX ".so")
INSTALL(TARGETS RemoteTreeViewerWrapper_pybind DESTINATION lib/python2.7/site-packages/)