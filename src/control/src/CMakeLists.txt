
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake/modules")

# find packages that don't work with normal find_package
find_package(Drake MODULE REQUIRED)
#find_package(Eigen REQUIRED)

  include_directories(SYSTEM
    ${DRAKE_INCLUDE_DIRS}
  )



message("eigen includ dirs ${EIGEN_INCLUDE_DIRS}")
message("cmake module path ${CMAKE_MODULE_PATH}")
find_package(lcm)
find_package(yaml-cpp)
find_package(robotlocomotion-lcmtypes)
#find_package(Eigen3)
#find_package(bot2-core)

if(Eigen3_FOUND)
    message("found Eigen3")
    message("eigen3 includ dirs ${EIGEN3_INCLUDE_DIR}")
endif()

#include_directories(SYSTEM
#    ${EIGEN3_INCLUDE_DIR}
#  )

message("DRAKE_INCLUDE_DIRS = ${DRAKE_INCLUDE_DIRS}")

# having some issue with pthread linking error so set some compile flag
#set(GCC_PTHREAD_FLAG "-pthread")
#SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_PTHREAD_FLAG}" )

add_executable(plan_runner plan_runner.cc)
target_link_libraries(plan_runner
    ${DRAKE_LIBRARIES}
    lcm
    pthread
    yaml-cpp
    robotlocomotion-lcmtypes
    Eigen3::Eigen
    )

target_link_libraries(plan_runner
    ${DRAKE_LIBRARIES}
    )


