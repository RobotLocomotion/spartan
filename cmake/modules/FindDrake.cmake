if(DRAKE_DIR)
  set(_include_dir_hint ${DRAKE_DIR}/include)
  set(_lib_dir_hint ${DRAKE_DIR}/lib)
  message("DRAKE_DIR is ${DRAKE_DIR}")
endif()

find_path(DRAKE_INCLUDE_DIR drake/util/drakeUtil.h HINTS ${_include_dir_hint} DOC "Drake source directory")

set(DRAKE_LIBRARIES)
set(_library_var_names)

macro(find_drake_library varName name doc)
  find_library(${varName} ${name} HINTS ${_lib_dir_hint} DOC ${doc})
  list(APPEND _library_var_names ${varName})
endmacro()

# add any additional drake libraries as needed
find_drake_library(DRAKE_MULTI_BODY_PARSERS_LIBRARY drakeMultibodyParsers "Drake Multibody Parsers library")
find_drake_library(DRAKE_RBM_LIBRARY drakeRBM "Drake RBM library")
find_drake_library(DRAKE_RIGID_BODY_PLANT_LIBRARY drakeRigidBodyPlant "Drake RigidBodyPlant library")
find_drake_library(DRAKE_COMMON_LIBRARY drakeCommon "Drake Common library")
find_drake_library(DRAKE_IKOPTIONS_LIBRARY drakeIKoptions "Drake IK Options library")
find_drake_library(DRAKE_RBMCONSTRAINT drakeRigidBodyConstraint "Drake Rigid Body Constraint library")
find_drake_library(DRAKE_IK_LIBRARY drakeIK "Drake IK library")
find_drake_library(DRAKE_JOINTS_LIBRARY drakeJoints "Drake Joints library")
find_drake_library(DRAKE_SHAPES_LIBRARY drakeShapes "Drake Shapes library")
find_drake_library(DRAKE_CHULL_LIBRARY drakeConvexHull "Drake Convex Hull library")
find_drake_library(DRAKE_GEOMETRYUTIL_LIBRARY drakeGeometryUtil "Drake Geometry Util library")
find_drake_library(DRAKE_KUKA_IIWA_ARM_COMMON_LIBRARY drakeKukaIiwaArmCommon "Drake KukaIiwaArmCommon library")
find_drake_library(DRAKE_LCM_LIBRARY drakeLcm "Drake lcm library")
find_drake_library(DRAKE_LCMUTIL_LIBRARY drakeLCMUtil "Drake LCM Util library")
find_drake_library(DRAKE_LCMTYPES_LIBRARY drake_lcmtypes "Drake LCM Types library")
find_drake_library(DRAKE_TRAJECTORIES_LIBRARY drakeTrajectories "Drake Trajectory library")
find_drake_library(DRAKE_YAMLUTIL_LIBRARY drakeYAMLUtil "Drake YAML utilities library")


if(NOT DRAKE_MULTI_BODY_PARSERS_LIBRARY)
  list(REMOVE_ITEM _library_var_names DRAKE_MULTI_BODY_PARSERS_LIBRARY)
endif()

if(NOT DRAKE_COMMON_LIBRARY)
  list(REMOVE_ITEM _library_var_names DRAKE_COMMON_LIBRARY)
endif()

if(NOT DRAKE_LCMUTIL_LIBRARY)
	message("Didn't find drakeLCMUtil")
endif()

if(DRAKE_YAMLUTIL_LIBRARY)
	message("found drakeYAMLUtil")
endif()

foreach(varName ${_library_var_names})
  list(APPEND DRAKE_LIBRARIES ${${varName}})
endforeach()




set(DRAKE_INCLUDE_DIRS
  ${DRAKE_INCLUDE_DIR}
)


# find some extra external libraries that are usually targeted by drake
find_package(Eigen3)
find_package(bot2-core) # this isn't a library so don't put it in DRAKE_EXTERNAL_LIBRARIES
find_package(yaml-cpp)
find_package(lcm)
find_package(robotlocomotion-lcmtypes)

set(DRAKE_EXTERNAL_LIBRARIES)
list(APPEND DRAKE_EXTERNAL_LIBRARIES "Eigen3::Eigen")
list(APPEND DRAKE_EXTERNAL_LIBRARIES "yaml-cpp")
list(APPEND DRAKE_EXTERNAL_LIBRARIES "lcm")
list(APPEND DRAKE_EXTERNAL_LIBRARIES "robotlocomotion-lcmtypes")


# this is where the drake lcmtypes live
list(APPEND DRAKE_INCLUDE_DIRS "${DRAKE_INCLUDE_DIR}/lcmtypes")


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Drake DEFAULT_MSG DRAKE_INCLUDE_DIR ${_library_var_names})
mark_as_advanced(DRAKE_INCLUDE_DIR DRAKE_EXTERNAL_LIBRARIES ${_library_var_names})
