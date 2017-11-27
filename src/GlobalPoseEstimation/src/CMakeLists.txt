cmake_minimum_required(VERSION 2.6.0)
cmake_policy(SET CMP0045 OLD)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -std=gnu++14")

include_directories(${CMAKE_INSTALL_PREFIX})
include_directories(${CMAKE_INSTALL_PREFIX}/include)
include_directories(${CMAKE_INSTALL_PREFIX}/include/lcmtypes)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIR}) 
message("Eigen3 include dir" ${EIGEN3_INCLUDE_DIR})

find_package(drake REQUIRED)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_library(point_cloud_generator point_cloud_generator.cpp point_cloud_generator.hpp)
target_link_libraries(point_cloud_generator lcm RemoteTreeViewerWrapper drakeMultibodyParsers drakeRBM drakeOptimization drakeShapes drakeJoints drakeCommon drakeLcm drakeRigidBodyPlant drake_lcmtypes yaml-cpp ${PCL_LIBRARIES} z pthread)
install(TARGETS point_cloud_generator DESTINATION lib)

add_executable(run_point_cloud_generator run_point_cloud_generator.cpp)
target_link_libraries(run_point_cloud_generator lcm RemoteTreeViewerWrapper point_cloud_generator drakeMultibodyParsers drakeRBM drakeShapes drakeJoints drakeCommon drakeLcm drakeRigidBodyPlant drake_lcmtypes yaml-cpp ${VTK_LIBRARIES} z pthread)
install(TARGETS run_point_cloud_generator DESTINATION bin)

add_executable(run_bogo_pose_estimator run_bogo_pose_estimator.cpp)
target_link_libraries(run_bogo_pose_estimator lcm RemoteTreeViewerWrapper drakeMultibodyParsers drakeRBM drakeShapes drakeJoints drakeCommon drakeLcm drakeRigidBodyPlant drake_lcmtypes yaml-cpp ${PCL_LIBRARIES} z pthread)
install(TARGETS run_bogo_pose_estimator DESTINATION bin)

add_library(mip_pose_estimator mip_pose_estimator.cpp mip_pose_estimator.hpp)
target_link_libraries(mip_pose_estimator lcm RemoteTreeViewerWrapper drakeMultibodyParsers drakeOptimization drakeRotationConstraint drakeRBM drakeShapes drakeJoints drakeCommon drakeLcm drakeRigidBodyPlant yaml-cpp z pthread)
install(TARGETS mip_pose_estimator DESTINATION lib)

add_executable(run_mip_pose_estimator run_mip_pose_estimator.cpp)
target_link_libraries(run_mip_pose_estimator lcm RemoteTreeViewerWrapper mip_pose_estimator point_cloud_generator drakeMultibodyParsers drakeOptimization drakeRotationConstraint drakeRBM drakeShapes drakeJoints drakeCommon drakeLcm drakeRigidBodyPlant drake_lcmtypes yaml-cpp ${VTK_LIBRARIES} z pthread)
install(TARGETS run_mip_pose_estimator DESTINATION bin)

add_executable(run_goicp_pose_estimator run_goicp_pose_estimator.cpp
    GoICP_V1.3/jly_goicp.cpp
    GoICP_V1.3/jly_3ddt.cpp
    GoICP_V1.3/matrix.cpp
    GoICP_V1.3/ConfigMap.cpp
    GoICP_V1.3/StringTokenizer.cpp)
target_link_libraries(run_goicp_pose_estimator lcm RemoteTreeViewerWrapper point_cloud_generator drakeMultibodyParsers drakeRBM drakeShapes drakeJoints drakeCommon drakeLcm drakeRigidBodyPlant drake_lcmtypes yaml-cpp ${VTK_LIBRARIES} z pthread)
install(TARGETS run_goicp_pose_estimator DESTINATION bin)

add_subdirectory(FastGlobalRegistration)

add_executable(run_fgr_pose_estimator run_fgr_pose_estimator.cpp)
target_link_libraries(run_fgr_pose_estimator FastGlobalRegistrationLib lcm RemoteTreeViewerWrapper point_cloud_generator drakeMultibodyParsers drakeRBM drakeShapes drakeJoints drakeCommon drakeLcm drakeRigidBodyPlant drake_lcmtypes yaml-cpp ${VTK_LIBRARIES} ${PCL_LIBRARIES} z pthread)
install(TARGETS run_fgr_pose_estimator DESTINATION bin)

include(ExternalProject)
set_property(DIRECTORY PROPERTY EP_PREFIX ${CMAKE_BINARY_DIR})
set(source_dir ${CMAKE_SOURCE_DIR})
set(build_dir ${CMAKE_BINARY_DIR})

ExternalProject_Add(Super4PCS
SOURCE_DIR ${source_dir}/src/Super4PCS
BINARY_DIR ${build_dir}/Super4PCS
BUILD_ALWAYS 1
CMAKE_CACHE_ARGS
  -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

find_package(OpenCV 2 REQUIRED)
add_executable(run_super4pcs_pose_estimator run_super4pcs_pose_estimator.cpp)
target_link_libraries(run_super4pcs_pose_estimator super4pcs_algo super4pcs_io super4pcs_accel chealpix lcm RemoteTreeViewerWrapper point_cloud_generator yaml-cpp ${VTK_LIBRARIES} ${OpenCV_LIBS} z pthread)
install(TARGETS run_super4pcs_pose_estimator DESTINATION bin)
add_dependencies(run_super4pcs_pose_estimator Super4PCS)

add_executable(crop_pointcloud_close_to_model crop_pointcloud_close_to_model.cpp)
target_link_libraries(crop_pointcloud_close_to_model lcm RemoteTreeViewerWrapper point_cloud_generator yaml-cpp ${VTK_LIBRARIES} ${PCL_LIBRARIES} drakeMath z pthread)
install(TARGETS crop_pointcloud_close_to_model DESTINATION bin)

add_library(eigen_histogram eigen_histogram.cpp eigen_histogram.hpp)
target_link_libraries(eigen_histogram)
install(TARGETS eigen_histogram DESTINATION lib)
add_executable(test_eigen_histogram test_eigen_histogram.cpp)
target_link_libraries(test_eigen_histogram eigen_histogram)
install(TARGETS test_eigen_histogram DESTINATION bin)

add_library(point_pair_features point_pair_features.cpp point_pair_features.hpp)
target_link_libraries(point_pair_features)
install(TARGETS point_pair_features DESTINATION lib)

add_executable(compute_point_pair_features compute_point_pair_features.cpp)
target_link_libraries(compute_point_pair_features point_pair_features lcm RemoteTreeViewerWrapper point_cloud_generator yaml-cpp ${VTK_LIBRARIES} ${PCL_LIBRARIES} drakeMath z pthread)
install(TARGETS compute_point_pair_features DESTINATION bin)

add_executable(compute_point_pair_feature_histogram compute_point_pair_feature_histogram.cpp)
target_link_libraries(compute_point_pair_feature_histogram eigen_histogram point_pair_features lcm RemoteTreeViewerWrapper point_cloud_generator yaml-cpp ${VTK_LIBRARIES} ${PCL_LIBRARIES} drakeMath z pthread)
install(TARGETS compute_point_pair_feature_histogram DESTINATION bin)

add_executable(run_ppf_histogram_matcher run_ppf_histogram_matcher.cpp)
target_link_libraries(run_ppf_histogram_matcher eigen_histogram point_pair_features lcm RemoteTreeViewerWrapper point_cloud_generator yaml-cpp ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} drakeMath z pthread)
install(TARGETS run_ppf_histogram_matcher DESTINATION bin)

add_executable(collect_point_pair_feature_histograms collect_point_pair_feature_histograms.cpp)
target_compile_options(collect_point_pair_feature_histograms PRIVATE -g)
target_link_libraries(collect_point_pair_feature_histograms yaml-cpp z pthread)
install(TARGETS collect_point_pair_feature_histograms DESTINATION bin)