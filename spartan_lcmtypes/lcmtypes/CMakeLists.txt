if(WIN32)
  set(CONFIG_INSTALL_DIR CMake)
else()
  set(CONFIG_INSTALL_DIR lib${LIB_SUFFIX}/${PROJECT_NAME}/cmake)
endif()

if(PYTHONINTERP_FOUND)
  set(python_args PYTHON_SOURCES python_install_sources)
endif()
if(JAVA_FOUND AND TARGET lcm::lcm-java)
  set(java_args JAVA_SOURCES java_sources)
endif()

lcm_wrap_types(
  C_EXPORT spartan_lcmtypes
  C_SOURCES c_sources
  C_HEADERS c_install_headers
  CPP_HEADERS cpp_install_headers
  CREATE_C_AGGREGATE_HEADER
  CREATE_CPP_AGGREGATE_HEADER
  ${python_args}
  ${java_args}
  ur10_command_t.lcm
)

lcm_add_library(lcmtypes_spartan-lcm C ${c_sources} ${c_install_headers})
generate_export_header(lcmtypes_spartan-lcm
  BASE_NAME spartan_lcmtypes)
target_include_directories(lcmtypes_spartan-lcm INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

lcm_add_library(lcmtypes_spartan-lcm-cpp CPP ${cpp_install_headers})
target_include_directories(lcmtypes_spartan-lcm-cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

lcm_install_headers(
  ${CMAKE_CURRENT_BINARY_DIR}/spartan_lcmtypes_export.h
  ${c_install_headers}
  ${cpp_install_headers}
  DESTINATION include/lcmtypes
)

install(TARGETS lcmtypes_spartan-lcm lcmtypes_spartan-lcm-cpp
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib${LIB_SUFFIX}
  ARCHIVE DESTINATION lib${LIB_SUFFIX}
  INCLUDES DESTINATION include/lcmtypes
)

if(PYTHONINTERP_FOUND)
  lcm_install_python(${python_install_sources})
endif()

if(JAVA_FOUND)
  if(NOT TARGET lcm::lcm-java)
    message(STATUS "lcm::lcm-java not found: not building Java LCM type bindings")
  else()
    add_jar(lcmtypes_spartan-lcm-jar
      OUTPUT_NAME lcmtypes_spartan-lcm
      INCLUDE_JARS lcm::lcm-java
      SOURCES ${java_sources}
    )

    install_jar(lcmtypes_spartan-lcm-jar share/java)

    set(SPARTAN_LCM_INCLUDE_JAVA
      "include(\${CMAKE_CURRENT_LIST_DIR}/spartan-lcmJavaTargets.cmake)"
    )
    export_jars(
      FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}JavaTargets.cmake
      TARGETS lcmtypes_spartan-lcm-jar
    )
    install_jar_exports(
      FILE ${PROJECT_NAME}JavaTargets.cmake
      DESTINATION ${CONFIG_INSTALL_DIR}
      TARGETS lcmtypes_spartan-lcm-jar
    )
  endif()
endif()

configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CONFIG_INSTALL_DIR}
)

# Exported targets for build directory
export(
  TARGETS lcmtypes_spartan-lcm
  FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
)

# Exported targets for installation
install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION ${CONFIG_INSTALL_DIR}
  FILE ${PROJECT_NAME}Targets.cmake
)

# Install package configuration files
install(FILES
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION ${CONFIG_INSTALL_DIR}
)
