
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../director/cmake/modules")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake/modules")

find_package(Drake MODULE REQUIRED)
find_package(Eigen REQUIRED)

  include_directories(SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${DRAKE_INCLUDE_DIRS}
  )

message("DRAKE_INCLUDE_DIRS = ${DRAKE_INCLUDE_DIRS}")


# having some issue with pthread linking error so set some compile flag
#set(GCC_PTHREAD_FLAG "-pthread")
#SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_PTHREAD_FLAG}" )

add_executable(iiwa_status_to_residual
    run_iiwa_status_to_residual.cc
    kuka_iiwa_status_to_residual.cc
    )

target_link_libraries(iiwa_status_to_residual
    ${DRAKE_LIBRARIES}
    lcm
    pthread
    yaml-cpp)

install(TARGETS iiwa_status_to_residual
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
