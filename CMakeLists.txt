cmake_minimum_required(VERSION 3.5)
project(spartan)

enable_testing()

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
    "The type of build. Options are: Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

# Build shared libraries by default
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# Set default install prefix
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING
    "Prefix for installation of sub-packages (note: required during build!)"
    FORCE)
endif()

# Configure the environment file
configure_file(config/setup_environment.sh.in ${CMAKE_BINARY_DIR}/setup_environment.sh @ONLY)

# Options
option(WITH_PERCEPTION "Build with perception libraries and tools." OFF)

# Setup external projects
include(ExternalProject)
set_property(DIRECTORY PROPERTY EP_PREFIX ${CMAKE_BINARY_DIR})
set(source_dir ${CMAKE_SOURCE_DIR})
set(build_dir ${CMAKE_BINARY_DIR})
include(cmake/set-python-args.cmake)

foreach(proj drake director signal-scope)
  if(NOT EXISTS ${source_dir}/${proj}/.git)
    message(FATAL_ERROR "\nThe ${proj} submodule is not available.\nPlease run: git submodule update --init")
  endif()
endforeach()

# Add external projects
ExternalProject_Add(drake
  SOURCE_DIR ${source_dir}/drake
  BINARY_DIR ${build_dir}/drake
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    -DDISABLE_MATLAB:BOOL=ON
    -DWITH_GUROBI:BOOL=OFF
    -DWITH_FCL:BOOL=OFF
    -DWITH_SNOPT:BOOL=ON
    -DWITH_DIRECTOR:BOOL=OFF
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  INSTALL_COMMAND ""
)

ExternalProject_Add(director
  SOURCE_DIR ${source_dir}/director/distro/superbuild
  BINARY_DIR ${build_dir}/director
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    -DUSE_EXTERNAL_INSTALL:BOOL=ON
    -DUSE_DRAKE:BOOL=ON
    -DUSE_LCM:BOOL=ON
    -DUSE_LIBBOT:BOOL=ON
    -DUSE_SYSTEM_EIGEN:BOOL=ON
    -DUSE_SYSTEM_LCM:BOOL=ON
    -DUSE_SYSTEM_LIBBOT:BOOL=ON
    -DUSE_PCL:BOOL=${WITH_PERCEPTION}
    -DUSE_APRILTAGS:BOOL=${WITH_PERCEPTION}
    -DUSE_KINECT:BOOL=${WITH_PERCEPTION}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  INSTALL_COMMAND ""
  DEPENDS drake
)

ExternalProject_Add(signal-scope
  SOURCE_DIR ${source_dir}/signal-scope
  BINARY_DIR ${build_dir}/signal-scope
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    ${python_args}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  DEPENDS director
)

ExternalProject_Add(ContactParticleFilter
  SOURCE_DIR ${source_dir}/src/ContactParticleFilter
  BINARY_DIR ${build_dir}/ContactParticleFilter
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  DEPENDS drake
)

# Add tests
add_subdirectory(tests)

