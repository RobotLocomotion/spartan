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
option(WITH_SNOPT "Build drake with support for the SNOPT solver." ON)

option(WITH_IIWA_DRIVER_RLG "Build drake-iiwa-driver, RLG version." ON)
option(WITH_IIWA_DRIVER_TRI "Build drake-iiwa-driver, TRI version." OFF)
option(WITH_SCHUNK_DRIVER "Build drake-schunk-driver." ON)
option(WITH_ROS "Build the catkin-workspace projects." ON)
option(WITH_REACHABILITY_ANALYZER "Build a reachability analyzer (better with snopt)" OFF)
option(WITH_TRIMESH "Build trimesh, a python utility for working with triangle meshes" OFF)


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
  -DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}
)

find_program(CP cp)
find_program(BAZEL bazel)
if(NOT BAZEL)
  message(FATAL_ERROR "Could NOT find bazel executable")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(BAZEL_COMPILATION_MODE dbg)
else()
  set(BAZEL_COMPILATION_MODE opt)
endif()
set(BAZEL_ENV CC=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER})
set(BAZEL_DRAKE_STARTUP_ARGS "--output_base=/$ENV{HOME}/.spartan-build/drake-build")
set(BAZEL_ARGS --compilation_mode=${BAZEL_COMPILATION_MODE} --config=unsupported_crosstool --cxxopt=-std=c++1y --host_cxxopt=-std=c++1y)
if(CMAKE_VERBOSE_MAKEFILE)
  set(BAZEL_ARGS --subcommands ${BAZEL_ARGS})
endif()
if(WITH_SNOPT)
  set(BAZEL_ARGS --config=snopt ${BAZEL_ARGS})
endif()

# Check that compiler is NOT ccache. Ccache is not compatible with bazel.
# "readlink -f" is not available on MacOS. We use a one-liner python script
# to get the result that "readlink -f" would give on Linux.
execute_process(
  COMMAND python -c "import os,sys; print(os.path.realpath(os.path.expanduser(sys.argv[1])))" "${CMAKE_CXX_COMPILER}"
  RESULT_VARIABLE COMPILER_RESULT
  OUTPUT_VARIABLE COMPILER_OUTPUT
  )
if(COMPILER_RESULT)
  message(FATAL_ERROR "Error while running the compiler ${CMAKE_CXX_COMPILER}.")
endif()
get_filename_component(COMPILER_NAME ${COMPILER_OUTPUT} NAME)
string(STRIP ${COMPILER_NAME} COMPILER_NAME)
if("${COMPILER_NAME}" STREQUAL "ccache")
  message(FATAL_ERROR
"Compiler is `ccache`. `ccache` is not compatible with `bazel` which is used to compile `drake`.
To use `ccache` to compile the rest of the project, use the CMake or environment variables \
CMAKE_C_COMPILER_LAUNCHER and CMAKE_CXX_COMPILER_LAUNCHER to set the path to `ccache`."
)
endif()


# Add external projects
# NOTE: For bazel < 0.5.2, two external projects that use Bazel must not build simultaneously.
# One can depend on another one, which will force the order in which they are built.
ExternalProject_Add(drake
    SOURCE_DIR ${source_dir}/drake
    BUILD_ALWAYS 1
    BUILD_IN_SOURCE 1
    USES_TERMINAL_BUILD 1
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ${BAZEL_ENV} "${BAZEL}" ${BAZEL_DRAKE_STARTUP_ARGS} build ${BAZEL_ARGS} //examples/kuka_iiwa_arm/... //:install @optitrack_driver//src:optitrack_client
    INSTALL_COMMAND ${BAZEL_ENV} "${BAZEL}" ${BAZEL_DRAKE_STARTUP_ARGS} run ${BAZEL_ARGS} //:install -- "${CMAKE_INSTALL_PREFIX}"
)
# This is based on drake's VTK/Qt dependency (see `tools/vtk.bzl` in drake).
# On MacOS, Drake uses homebrew VTK compiled with Qt5 and brew Python
# On Linux, checks distribution
# Ubuntu 14.04: Drake downloads a precompiled version of VTK compiled with Qt4
# Ubuntu 16.04: Drake downloads a precompiled version of VTK compiled with Qt5
# Otherwise, assume Qt5
set(USED_QTVERSION 5)
if(APPLE)
  # VTK version used by drake
  set(VTK_INSTALL_DIR /usr/local/opt/vtk@8.0)
  set(VTK_ARGS
    -DVTK_DIR:PATH=${VTK_INSTALL_DIR}/lib/cmake/vtk-8.0
    -DPYTHON_CONFIG_EXECUTABLE:FILEPATH=/usr/local/bin/python2.7-config
  )
  set(QT_ARGS -DQt5_DIR:PATH=/usr/local/opt/qt/lib/cmake/Qt5)
  set(VTK_PYTHON_DIR ${VTK_INSTALL_DIR}/lib/python2.7/site-packages)
elseif(UNIX)
  find_program(SED sed)
  if(NOT SED)
    message(FATAL_ERROR "Could NOT find sed executable")
  endif()
  execute_process(COMMAND "${SED}" -n "/^\\(NAME\\|VERSION_ID\\)=/{s/[^=]*=//;s/\"//g;p}" /etc/os-release
    RESULT_VARIABLE SED_RESULT
    OUTPUT_VARIABLE SED_OUTPUT
  )
  if(SED_RESULT)
    message(FATAL_ERROR "Could NOT determine Linux distribution information")
  endif()
 string(REGEX REPLACE "\n" " " LINUX_DISTRIBUTION "${SED_OUTPUT}")
 string(STRIP ${LINUX_DISTRIBUTION} LINUX_DISTRIBUTION)
 if(LINUX_DISTRIBUTION STREQUAL "Ubuntu 14.04")
   set(USED_QTVERSION 4)
 endif()
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
    -DUSE_APRILTAGS:BOOL=${WITH_PERCEPTION}
    -DUSE_OPENNI2_LCM:BOOL=OFF # don't build OPENNI2 driver, conflicts with openni2_launch in ROS
    -DUSE_PERCEPTION:BOOL=${WITH_PERCEPTION}
    ${CMAKE_ARGS_FOR_EXTERNALS}
  INSTALL_COMMAND ""
  DEPENDS drake
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

ExternalProject_Add(RemoteTreeViewer
  SOURCE_DIR ${source_dir}/src/RemoteTreeViewer
  BINARY_DIR ${build_dir}/RemoteTreeViewer
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    ${CMAKE_ARGS_FOR_EXTERNALS}
  DEPENDS drake
)

if (WITH_REACHABILITY_ANALYZER)
  ExternalProject_Add(ReachabilityAnalyzer
  SOURCE_DIR ${source_dir}/src/ReachabilityAnalyzer
  BINARY_DIR ${build_dir}/ReachabilityAnalyzer
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    ${CMAKE_ARGS_FOR_EXTERNALS}
  DEPENDS drake director RemoteTreeViewer common_utils
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
        INSTALL_COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/bin/ && ${CP} ${build_dir}/externals/v-hacd/bin/linux/testVHACD ${CMAKE_INSTALL_PREFIX}/bin/testVHACD
      )
    else()
      message(FATAL_ERROR "Not using linux, don't know how to install v-hacd.")
    endif()

    SET(TRIMESH_GIT_REPOSITORY git@github.com:gizatt/trimesh.git)
    SET(TRIMESH_GIT_TAG 1f6d10850d5a1f667385f3776d0b9d71df92fac3)
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
      INSTALL_COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages && ${CP} -r ${build_dir}/externals/trimesh/build/lib.linux-x86_64-2.7/trimesh ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/
      BUILD_IN_SOURCE 1
      DEPENDS v-hacd
    )
endif()

if (OFF)
  ExternalProject_Add(ObjectDetection
    SOURCE_DIR ${source_dir}/src/ObjectDetection
    BINARY_DIR ${build_dir}/ObjectDetection
    BUILD_ALWAYS 1
    CMAKE_CACHE_ARGS
      ${CMAKE_ARGS_FOR_EXTERNALS}
    DEPENDS director RemoteTreeViewer
  )

endif()

if (WITH_IIWA_DRIVER_RLG OR WITH_IIWA_DRIVER_TRI)
  set(IIWA_DRIVER_GIT_REPOSITORY
	"git@github.com:RobotLocomotion/drake-iiwa-driver.git"
	CACHE STRING
	"Location of drake-iiwa-driver repo including kuka-fri sources")

  # initialize IIWA_DRIVER_GIT_TAG to be empty
  # then set it based on if we are using RLG or TRI driver
  set(IIWA_DRIVER_GIT_TAG "")

  if (WITH_IIWA_DRIVER_RLG)
    set(IIWA_DRIVER_GIT_TAG
    "22e97d4db4057c65918deb70dec2323e4fe81191"
    )
  endif()

  if (WITH_IIWA_DRIVER_TRI)
    set(IIWA_DRIVER_GIT_TAG
    "1b11832"
    )
  endif()

 #message("IIWA_DRIVER_GIT_TAG ${IIWA_DRIVER_GIT_TAG}")


  ExternalProject_Add(drake-iiwa-driver
    SOURCE_DIR ${build_dir}/externals/drake-iiwa-driver
    BINARY_DIR ${build_dir}/drake-iiwa-driver
    GIT_REPOSITORY "${IIWA_DRIVER_GIT_REPOSITORY}"
    GIT_TAG "${IIWA_DRIVER_GIT_TAG}"
    BUILD_ALWAYS 1
    CMAKE_CACHE_ARGS
      ${CMAKE_ARGS_FOR_EXTERNALS}
    DEPENDS drake
  )

endif()

if (WITH_SCHUNK_DRIVER)

  ExternalProject_Add(drake-schunk-driver
    SOURCE_DIR ${build_dir}/externals/drake-schunk-driver
    GIT_REPOSITORY git@github.com:RobotLocomotion/drake-schunk-driver.git
    GIT_TAG a28d35c
    BUILD_ALWAYS 1
    BUILD_IN_SOURCE 1
    USES_TERMINAL_BUILD 1
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ${BAZEL_ENV} "${BAZEL}" build //...
    INSTALL_COMMAND
      mkdir -p ${build_dir}/install/bin/ && "${CP}" ${build_dir}/externals/drake-schunk-driver/bazel-bin/src/schunk_driver ${CMAKE_INSTALL_PREFIX}/bin/schunk_driver &&
      chmod ug+w ${CMAKE_INSTALL_PREFIX}/bin/schunk_driver
    DEPENDS drake
  )

endif()

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
    DEPENDS drake director common_utils RemoteTreeViewer
  )
endif()

# Configure the environment file
configure_file(config/setup_environment.sh.in ${CMAKE_BINARY_DIR}/setup_environment.sh @ONLY)

# Add tests
add_subdirectory(tests)
