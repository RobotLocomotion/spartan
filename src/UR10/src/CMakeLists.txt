list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake/modules")

# find packages that don't work with normal find_package
find_package(Drake MODULE REQUIRED)

  include_directories(SYSTEM
    ${DRAKE_INCLUDE_DIRS}
  )

#find_package(lcm)
#find_package(yaml-cpp)




add_executable(rescale_trajectory
    rescale_trajectory.cpp
    rescale_trajectory.hpp
    )

target_link_libraries(rescale_trajectory
    ${DRAKE_LIBRARIES}
    ${DRAKE_EXTERNAL_LIBRARIES}
    pthread
    )

install(TARGETS rescale_trajectory
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

add_subdirectory(control)