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

# Options
option(WITH_PERCEPTION "Build with perception libraries and tools." ON)
option(WITH_ROBOTLOCOMOTION_SNOPT "Build drake with support for the SNOPT solver." ON)
option(WITH_GUROBI "Build with support for Gurobi" OFF)
option(WITH_MOSEK "Build with support for MOSEK" OFF)

# Intended to install the SAME version as Drake uses internally.
# However, the SHAs are currently not synced (I don't know how to
# write that kind of bazel query).
option(WITH_BULLET3 "Build a complete version of Bullet, and install pybullet support." OFF)

option(WITH_ROS "Build the catkin-workspace projects." ON)
option(WITH_REACHABILITY_ANALYZER "Build a reachability analyzer (better with snopt)" OFF)
option(WITH_TRIMESH "Build trimesh, a python utility for working with triangle meshes" OFF)
option(WITH_DRAKE_IIWA_SIM "Build IIWA full system simulation with Drake tools?" OFF)

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

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

set(CMAKE_ARGS_FOR_EXTERNALS
  -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
  "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
  "-DCMAKE_C_COMPILER_LAUNCHER:FILEPATH=${CMAKE_C_COMPILER_LAUNCHER}"
  "-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}"
  "-DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}"
  "-DCMAKE_CXX_COMPILER_LAUNCHER:FILEPATH=${CMAKE_CXX_COMPILER_LAUNCHER}"
  "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}"
  "-DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}"
  "-DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}"
  "-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}"
  "-DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}"
  "-DCMAKE_SHARED_LINKER_FLAGS:STRING=${CMAKE_SHARED_LINKER_FLAGS}"
  "-DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}"
  "-DCMAKE_MODULE_PATH:STRING=${CMAKE_MODULE_PATH}"
)

ExternalProject_Add(drake
    SOURCE_DIR ${source_dir}/drake
    BINARY_DIR ${build_dir}/drake
    BUILD_ALWAYS 1
    CMAKE_CACHE_ARGS
      -DWITH_ROBOTLOCOMOTION_SNOPT:BOOL=${WITH_ROBOTLOCOMOTION_SNOPT}
      -DWITH_SNOPT:BOOL=OFF
      -DWITH_GUROBI:BOOL=${WITH_GUROBI}
      -DWITH_MOSEK:BOOL=${WITH_MOSEK}
      ${CMAKE_ARGS_FOR_EXTERNALS}
    INSTALL_COMMAND ""
)


set(LCM_PYTHON_DIR_HINTS "${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages")

ExternalProject_Add(libbot
    SOURCE_DIR ${build_dir}/externals/libbot
    BINARY_DIR ${build_dir}/libbot
    GIT_REPOSITORY "https://github.com/RobotLocomotion/libbot2.git"
    GIT_TAG "b47cc520aaf8f97c7cfcf370827b3345bca061c3"
    BUILD_ALWAYS 1
    CMAKE_CACHE_ARGS
      -DLCM_PYTHON_DIR_HINTS:FILEPATH=${LCM_PYTHON_DIR_HINTS}
      ${CMAKE_ARGS_FOR_EXTERNALS}
    DEPENDS drake
)

set(USED_QTVERSION 5)
# On MacOS, Drake uses homebrew VTK compiled with Qt5 and brew Python
if(APPLE)
  # VTK version used by drake
  set(VTK_INSTALL_DIR /usr/local/opt/vtk@8.0)
  # To configure `director`
  set(VTK_ARGS
    -DVTK_DIR:PATH=${VTK_INSTALL_DIR}/lib/cmake/vtk-8.0
    -DPYTHON_CONFIG_EXECUTABLE:FILEPATH=/usr/local/bin/python2.7-config
  )
  # To configure `config/setup_environment.sh.in`
  set(VTK_PYTHON_DIR ${VTK_INSTALL_DIR}/lib/python2.7/site-packages)
  # For signal-scope
  set(QT_ARGS -DQt5_DIR:PATH=/usr/local/opt/qt/lib/cmake/Qt5)
endif()

ExternalProject_Add(director
  SOURCE_DIR ${source_dir}/director/distro/superbuild
  BINARY_DIR ${build_dir}/director
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    -DUSE_EXTERNAL_INSTALL:BOOL=ON
    -DDD_QT_VERSION:STRING=${USED_QTVERSION}
    -DUSE_DRAKE:BOOL=ON
    -DUSE_LCM:BOOL=ON
    -DUSE_LIBBOT:BOOL=ON
    -DUSE_SYSTEM_EIGEN:BOOL=ON
    -DUSE_SYSTEM_LCM:BOOL=ON
    -DUSE_SYSTEM_LIBBOT:BOOL=ON
    -DUSE_SYSTEM_VTK:BOOL=ON
    ${VTK_ARGS}
    -DUSE_PCL:BOOL=${WITH_PERCEPTION}
    -DUSE_APRILTAGS:BOOL=OFF
    -DUSE_OPENNI2_LCM:BOOL=OFF # don't build OPENNI2 driver, conflicts with openni2_launch in ROS
    -DUSE_PERCEPTION:BOOL=${WITH_PERCEPTION}
    ${CMAKE_ARGS_FOR_EXTERNALS}
  INSTALL_COMMAND ""
  DEPENDS libbot drake
)

ExternalProject_Add(pybind11
  SOURCE_DIR ${build_dir}/externals/pybind11
  BINARY_DIR ${build_dir}/pybind11
  GIT_REPOSITORY git@github.com:pybind/pybind11
  GIT_TAG v2.2.1
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    ${CMAKE_ARGS_FOR_EXTERNALS}
    -DPYBIND11_INSTALL:BOOL=ON
    -DPYBIND11_TEST:BOOL=OFF
    -DPYBIND11_PYTHON_VERSION:STRING=2.7
)

ExternalProject_Add(common_utils
  SOURCE_DIR ${source_dir}/src/common_utils
  BINARY_DIR ${build_dir}/common_utils
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    ${CMAKE_ARGS_FOR_EXTERNALS}
)

ExternalProject_Add(signal-scope
  SOURCE_DIR ${source_dir}/signal-scope
  BINARY_DIR ${build_dir}/signal-scope
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    ${python_args}
    -DUSED_QTVERSION:STRING=${USED_QTVERSION}
    ${QT_ARGS}
    ${CMAKE_ARGS_FOR_EXTERNALS}
  DEPENDS director
)

ExternalProject_Add(ContactParticleFilter
  SOURCE_DIR ${source_dir}/src/ContactParticleFilter
  BINARY_DIR ${build_dir}/ContactParticleFilter
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    ${CMAKE_ARGS_FOR_EXTERNALS}
  DEPENDS drake
)

ExternalProject_Add(spartan_lcmtypes
  SOURCE_DIR ${source_dir}/spartan_lcmtypes
  BINARY_DIR ${build_dir}/spartan_lcmtypes
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    ${CMAKE_ARGS_FOR_EXTERNALS}
  DEPENDS drake
)

# ExternalProject_Add(RemoteTreeViewer
#   SOURCE_DIR ${source_dir}/src/RemoteTreeViewer
#   BINARY_DIR ${build_dir}/RemoteTreeViewer
#   BUILD_ALWAYS 1
#   CMAKE_CACHE_ARGS
#     ${CMAKE_ARGS_FOR_EXTERNALS}
#   DEPENDS drake
# )

# if (WITH_REACHABILITY_ANALYZER)
#   ExternalProject_Add(ReachabilityAnalyzer
#   SOURCE_DIR ${source_dir}/src/ReachabilityAnalyzer
#   BINARY_DIR ${build_dir}/ReachabilityAnalyzer
#   BUILD_ALWAYS 1
#   CMAKE_CACHE_ARGS
#     ${CMAKE_ARGS_FOR_EXTERNALS}
#   DEPENDS drake director RemoteTreeViewer common_utils
#   )
# endif()

if (WITH_BULLET3)
  set(BULLET3_GIT_REPOSITORY
  "git@github.com:bulletphysics/bullet3.git"
  CACHE STRING
  "Location of bullet3 repo.")

  # Different than Drake's version -- see
  # spartan/drake/WORKSPACE for Drake's version (2.86.1,
  # as of writing)
  # We need something recent enough to have exact python
  # version matching supported.
  set(BULLET3_GIT_TAG "c0619ac92408788d7fa72fa787b3b47ab08b5d2d")

  # Extra arguments lifted from bullet install helper:
  # https://github.com/bulletphysics/bullet3/blob/master/build_cmake_pybullet_double.sh
  # I'm making this depend on Drake so it's guaranteed to
  # install after Drake. We're installing a different
  # version than Drake (2.86.1 vs post-2.87), which is risky
  # enough already.
  ExternalProject_Add(bullet3
    SOURCE_DIR ${build_dir}/externals/bullet3
    BINARY_DIR ${build_dir}/bullet3
    GIT_REPOSITORY "${BULLET3_GIT_REPOSITORY}"
    GIT_TAG "${BULLET3_GIT_TAG}"
    BUILD_ALWAYS 1
    CMAKE_CACHE_ARGS
      ${CMAKE_ARGS_FOR_EXTERNALS}
      -DBUILD_PYBULLET:BOOL=ON
      -DBUILD_PYBULLET_NUMPY:BOOL=ON
      -DPYTHON_VERSION_PYBULLET:STRING=2.7
      -DEXACT_PYTHON_VERSION:BOOL=ON
      -DUSE_DOUBLE_PRECISION:BOOL=ON
    INSTALL_COMMAND make install && mkdir -p ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/ && cp ${build_dir}/bullet3/examples/pybullet/pybullet.so ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages
    DEPENDS drake
  )

endif()

if (WITH_TRIMESH)
    find_package(PythonInterp REQUIRED)

    # Grab and copy in the v-hacd executable
    # (which is precompiled for ubuntu 16.04
    # on this branch)
    if (UNIX AND NOT APPLE)
      SET(VHACD_GIT_REPOSITORY git@github.com:mikedh/v-hacd-1.git)
      SET(VHACD_GIT_TAG e3acd8637397fab37423c734c6a980d7031d772f)
      ExternalProject_Add(v-hacd
        SOURCE_DIR ${build_dir}/externals/v-hacd
        BINARY_DIR ${build_dir}/v-hacd
        GIT_REPOSITORY "${VHACD_GIT_REPOSITORY}"
        GIT_TAG "${VHACD_GIT_TAG}"
        BUILD_ALWAYS 1
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/bin/ && cp ${build_dir}/externals/v-hacd/bin/linux/testVHACD ${CMAKE_INSTALL_PREFIX}/bin/testVHACD
      )
    else()
      message(FATAL_ERROR "Not using linux, don't know how to install v-hacd.")
    endif()

    SET(TRIMESH_GIT_REPOSITORY git@github.com:mikedh/trimesh.git)
    SET(TRIMESH_GIT_TAG 2.35.15)
    # Installs our customized version of trimesh
    # to the project's site-packages build folder.
    # I use the setup.py to run the build step,
    # but do the module installation manually,
    # as it was giving me trouble...
    ExternalProject_Add(trimesh
      SOURCE_DIR ${build_dir}/externals/trimesh
      CONFIGURE_COMMAND ""
      GIT_REPOSITORY "${TRIMESH_GIT_REPOSITORY}"
      GIT_TAG "${TRIMESH_GIT_TAG}"
      BUILD_ALWAYS 1
      BUILD_COMMAND python setup.py build
      INSTALL_COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages && cp -r ${build_dir}/externals/trimesh/build/lib.linux-x86_64-2.7/trimesh ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/
      BUILD_IN_SOURCE 1
      DEPENDS v-hacd
    )
endif()

# if (OFF)
#   ExternalProject_Add(ObjectDetection
#     SOURCE_DIR ${source_dir}/src/ObjectDetection
#     BINARY_DIR ${build_dir}/ObjectDetection
#     BUILD_ALWAYS 1
#     CMAKE_CACHE_ARGS
#       ${CMAKE_ARGS_FOR_EXTERNALS}
#     DEPENDS director RemoteTreeViewer
#   )

# endif()


if (WITH_ROS)
  if ((NOT DEFINED ENV{CMAKE_PREFIX_PATH}) OR (NOT $ENV{CMAKE_PREFIX_PATH} MATCHES ".*ros.*"))
    message(FATAL_ERROR "\nWITH_ROS is set, but I don't see ROS in your CMAKE_PREFIX_PATH. Did you call USE_ROS in your terminal?")
  endif()

  string(REPLACE ":" ";" CMAKE_PREFIX_PATH_ENV $ENV{CMAKE_PREFIX_PATH})

  ExternalProject_Add(catkin-projects
    SOURCE_DIR ${source_dir}/src/catkin_projects
    BINARY_DIR ${build_dir}/catkin_projects
    BUILD_ALWAYS 1
    CMAKE_CACHE_ARGS
      ${CMAKE_ARGS_FOR_EXTERNALS}
      # Combine environment prefix path (with ROS) with our own
      "-DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX};${CMAKE_PREFIX_PATH_ENV}"
      -DWITH_DRAKE_IIWA_SIM:BOOL=${WITH_DRAKE_IIWA_SIM}
      -Drazer_hydra_INSTALL_UDEV_RULES:BOOL=OFF
    DEPENDS drake director common_utils
  )
endif()

# Configure the environment file
configure_file(config/setup_environment.sh.in ${CMAKE_BINARY_DIR}/setup_environment.sh @ONLY)

# Add tests
add_subdirectory(tests)
