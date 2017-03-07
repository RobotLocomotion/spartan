find_package(drake)

add_executable(iiwa_status_to_residual
    run_iiwa_status_to_residual.cc
    kuka_iiwa_status_to_residual.cc
    )

target_link_libraries(iiwa_status_to_residual
    drakeCommon
    drakeLCMUtil
    lcm
    yaml-cpp)

install(TARGETS iiwa_status_to_residual
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
