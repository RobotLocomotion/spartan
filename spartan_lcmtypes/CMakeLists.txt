project(spartan-lcm)
cmake_minimum_required(VERSION 3.1)

if(CMAKE_VERSION VERSION_LESS 3.7)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
endif()

include(CMakePackageConfigHelpers)
include(GenerateExportHeader)

find_package(PythonInterp)
find_package(Java)

if(JAVA_FOUND)
  include(UseJava)
endif()

# Enable ELF hidden visibility
set(CMAKE_C_VISIBILITY_PRESET "hidden")
set(CMAKE_CXX_VISIBILITY_PRESET "hidden")
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

if(POLICY CMP0063)
  cmake_policy(SET CMP0063 NEW)
endif()

# Enable PIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# TODO remove when minimum CMake >= 3.7
if(CMAKE_VERSION VERSION_LESS 3.7)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
endif()

find_package(lcm REQUIRED)
include(${LCM_USE_FILE})

add_subdirectory(lcmtypes)
add_subdirectory(pkgconfig)
