cmake_minimum_required(VERSION 3.5)
project(SceneScanner)
set(CMAKE_CXX_STANDARD 11)

find_package(drake REQUIRED)
find_package(gflags REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

link_directories(${CMAKE_INSTALL_PREFIX}/lib)

add_library(phong_vertex_shader phong_vertex_shader.cpp phong_vertex_shader.h)
target_link_libraries(phong_vertex_shader
	${drake_LIBRARIES}
    ${gflags_LIBRARIES})
	
add_executable(run_scene_scanner run_scene_scanner.cpp)
target_link_libraries(run_scene_scanner
    z
    pthread
    ${drake_LIBRARIES}
    ${gflags_LIBRARIES}
    ${VTK_LIBRARIES}
    RemoteTreeViewerWrapper
    phong_vertex_shader)

install(TARGETS run_scene_scanner DESTINATION bin)
install(TARGETS phong_vertex_shader DESTINATION lib)
