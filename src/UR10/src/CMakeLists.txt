find_package(PythonLibs REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake/modules")
set(PYDRAKE_INSTALL_DIR lib/python2.7/dist-packages/pydrake)

find_package(pybind11)

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

pybind11_add_module(py_rescale py_rescale.cc)
target_link_libraries(py_rescale PRIVATE ${EIGEN_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${DRAKE_LIBRARIES})
#target_compile_options(_py_rescale PRIVATE "-fvisibility=default")
install(TARGETS py_rescale DESTINATION /usr/local/lib/python2.7/dist-packages/)
