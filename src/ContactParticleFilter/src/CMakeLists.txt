add_executable(iiwa_status_to_residual
    run_iiwa_status_to_residual.cc
    kuka_iiwa_status_to_residual.cc
    )

target_link_libraries(iiwa_status_to_residual
    Eigen3::Eigen
    lcm
    drakeLCMUtil
    drakeCommon
    yaml-cpp
    robotlocomotion-lcmtypes-cpp)