find_package(PythonLibs REQUIRED)
set(PYDRAKE_INSTALL_DIR lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages/pydrake)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake/modules")

# find packages that don't work with normal find_package
find_package(Drake MODULE REQUIRED)

  include_directories(SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${DRAKE_INCLUDE_DIRS}
  )
message("DRAKE_INCLUDE_DIRS = ${DRAKE_INCLUDE_DIRS}")

#string(REPLACE "-Wl,--no-undefined" "" CMAKE_MODULE_LINKER_FLAGS ${CMAKE_MODULE_LINKER_FLAGS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-shadow")
#include_directories("${NUMPY_INCLUDE_DIRS}")


pybind11_add_module(_py_rescale py_rescale.cc)
target_link_libraries(_py_rescale PRIVATE ${EIGEN_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${DRAKE_LIBRARIES})
target_compile_options(_py_rescale PRIVATE "-fvisibility=default")
install(TARGETS _py_rescale DESTINATION ${PYDRAKE_INSTALL_DIR})
