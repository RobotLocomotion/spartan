if(DRAKE_DIR)
  set(_include_dir_hint ${DRAKE_DIR}/include)
  set(_lib_dir_hint ${DRAKE_DIR}/lib)
  message("DRAKE_DIR is ${DRAKE_DIR}")
endif()

message("Lucas message")
message("_lib_dir_hint = ${_lib_dir_hint}")


find_path(DRAKE_INCLUDE_DIR drake/util/drakeUtil.h HINTS ${_include_dir_hint} DOC "Drake source directory")

message("DRAKE_INCLUDE_DIR = ${DRAKE_INCLUDE_DIR}")

set(DRAKE_LIBRARIES)
set(_library_var_names)

macro(find_drake_library varName name doc)
  find_library(${varName} ${name} HINTS ${_lib_dir_hint} DOC ${doc})
  list(APPEND _library_var_names ${varName})
endmacro()



find_drake_library(DRAKE_MULTI_BODY_PARSERS_LIBRARY drakeMultibodyParsers "Drake Multibody Parsers library")
find_drake_library(DRAKE_RBM_LIBRARY drakeRBM "Drake RBM library")
find_drake_library(DRAKE_COMMON_LIBRARY drakeCommon "Drake Common library")
find_drake_library(DRAKE_JOINTS_LIBRARY drakeJoints "Drake Joints library")
find_drake_library(DRAKE_SHAPES_LIBRARY drakeShapes "Drake Shapes library")
find_drake_library(DRAKE_CHULL_LIBRARY drakeConvexHull "Drake Convex Hull library")
find_drake_library(DRAKE_GEOMETRYUTIL_LIBRARY drakeGeometryUtil "Drake Geometry Util library")
find_drake_library(DRAKE_LCMUTIL_LIBRARY drakeLCMUtil "Drake LCM Util library")
find_drake_library(DRAKE_LCMTYPES_LIBRARY drake_lcmtypes "Drake LCM Types library")
# find_drake_library(YAML_CPP_LIBRARY yaml-cpp "yaml-cpp utilities library")
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

# this is where the drake lcmtypes live
list(APPEND DRAKE_INCLUDE_DIRS "${DRAKE_INCLUDE_DIR}/lcmtypes")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Drake DEFAULT_MSG DRAKE_INCLUDE_DIR ${_library_var_names})
mark_as_advanced(DRAKE_INCLUDE_DIR ${_library_var_names})
