list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake/modules")

# find packages that don't work with normal find_package
find_package(Drake MODULE REQUIRED)
find_package(Eigen REQUIRED)

  include_directories(SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${DRAKE_INCLUDE_DIRS}
  )

find_package(lcm)
find_package(yaml-cpp)

message("DRAKE_INCLUDE_DIRS = ${DRAKE_INCLUDE_DIRS}")


add_executable(rescale_trajectory
    rescale_trajectory.cpp
    rescale_trajectory.hpp
    )

target_link_libraries(rescale_trajectory
    ${DRAKE_LIBRARIES}
    lcm
    pthread
    yaml-cpp)

install(TARGETS rescale_trajectory
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
