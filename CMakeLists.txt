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
option(WITH_PERCEPTION "Build with perception libraries and tools." ON)

option(WITH_IIWA_DRIVER "Build drake-iiwa-driver." OFF)
option(WITH_SCHUNK_DRIVER "Build drake-schunk-driver." OFF)
option(WITH_OPTITRACK_DRIVER "Build optitrack-driver." OFF)

option(BUILD_GYGES "Build the GYGES external project" OFF)
option(USE_SYSTEM_VTK "Use system VTK for drake and director." OFF)


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
    -DUSE_SYSTEM_VTK:BOOL=${USE_SYSTEM_VTK}
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
    -DUSE_SYSTEM_VTK:BOOL=ON
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

ExternalProject_Add(spartan_lcmtypes
  SOURCE_DIR ${source_dir}/spartan_lcmtypes
  BINARY_DIR ${build_dir}/spartan_lcmtypes
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

ExternalProject_Add(RemoteTreeViewer
  SOURCE_DIR ${source_dir}/src/RmefaoteTreeViewer
  BINARY_DIR ${build_dir}/RemoteTreeViewer
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DCMAKE_MODULE_PATH:STRING=${source_dir}/drake/cmake/modules
  DEPENDS drake
)

if (WITH_PERCEPTION)
  ExternalProject_Add(PointCloudInference
    SOURCE_DIR ${source_dir}/src/PointCloudInference
    BINARY_DIR ${build_dir}/PointCloudInference
    BUILD_ALWAYS 1
    CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
      -DCMAKE_MODULE_PATH:STRING=${source_dir}/drake/cmake/modules
    DEPENDS drake RemoteTreeViewer
    INSTALL_COMMAND ""
  )
endif()

if (BUILD_GYGES)
  ExternalProject_Add(GYGES
    SOURCE_DIR ${source_dir}/src/Gyges
    BINARY_DIR ${build_dir}/Gyges
    BUILD_ALWAYS 1
    CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    DEPENDS drake spartan_lcmtypes
  )
endif()


if (OFF)
  ExternalProject_Add(ObjectDetection
    SOURCE_DIR ${source_dir}/src/ObjectDetection
    BINARY_DIR ${build_dir}/ObjectDetection
    BUILD_ALWAYS 1
    CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
      -DCMAKE_MODULE_PATH:STRING=${source_dir}/drake/cmake/modules
    DEPENDS drake director RemoteTreeViewer
  )

endif()

if (WITH_IIWA_DRIVER)

  set(IIWA_DRIVER_GIT_REPOSITORY
	"git@github.com:RobotLocomotion/drake-iiwa-driver.git"
	CACHE STRING
	"Location of drake-iiwa-driver repo including kuka-fri sources")
  set(IIWA_DRIVER_GIT_TAG
	"19e20aa"
	CACHE STRING
	"Build tag of drake-iiwa-driver repo")

  ExternalProject_Add(drake-iiwa-driver
    SOURCE_DIR ${build_dir}/externals/drake-iiwa-driver
    BINARY_DIR ${build_dir}/drake-iiwa-driver
    GIT_REPOSITORY "${IIWA_DRIVER_GIT_REPOSITORY}"
    GIT_TAG "${IIWA_DRIVER_GIT_TAG}"
    BUILD_ALWAYS 1
    CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
      -DCMAKE_MODULE_PATH:STRING=${source_dir}/drake/cmake/modules
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
    CONFIGURE_COMMAND ""
    BUILD_COMMAND bazel build //...
    INSTALL_COMMAND ""
    DEPENDS drake
  )

endif()

if (WITH_OPTITRACK_DRIVER)

  ExternalProject_Add(optitrack-driver
    SOURCE_DIR ${build_dir}/externals/optitrack-driver
    GIT_REPOSITORY https://github.com/sammy-tri/optitrack-driver.git
    GIT_TAG e78b74c
    BUILD_ALWAYS 1
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND ""
    BUILD_COMMAND bazel build //...
    INSTALL_COMMAND ./install-optitrack.sh ${CMAKE_INSTALL_PREFIX}
  )

endif()

# Add tests
add_subdirectory(tests)
