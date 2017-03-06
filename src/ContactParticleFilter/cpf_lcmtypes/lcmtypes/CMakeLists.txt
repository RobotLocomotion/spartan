if(WIN32)
  set(CONFIG_INSTALL_DIR CMake)
else()
  set(CONFIG_INSTALL_DIR lib${LIB_SUFFIX}/${PROJECT_NAME}/cmake)
endif()

if(PYTHONINTERP_FOUND)
  set(python_args PYTHON_SOURCES python_install_sources)
endif()
if(JAVA_FOUND AND TARGET lcm-java)
  set(java_args JAVA_SOURCES java_sources)
endif()

lcm_wrap_types(
  C_EXPORT cpf_lcmtypes
  C_SOURCES c_sources
  C_HEADERS c_install_headers
  CPP_HEADERS cpp_install_headers
  CREATE_C_AGGREGATE_HEADER
  CREATE_CPP_AGGREGATE_HEADER
  ${python_args}
  ${java_args}
  actual_and_estimated_contact_locations_t.lcm
  contact_filter_body_wrench_estimate_t.lcm
  contact_filter_estimate_t.lcm
  CPF_data_t.lcm
  CPF_particle_set_t.lcm
  CPF_particle_t.lcm
  external_force_torque_t.lcm
  multiple_contact_location_t.lcm
  residual_observer_state_t.lcm
  single_contact_filter_estimate_t.lcm
)

lcm_add_library(lcmtypes_cpf-lcm C ${c_sources} ${c_install_headers})
generate_export_header(lcmtypes_cpf-lcm
  BASE_NAME cpf_lcmtypes)
target_include_directories(lcmtypes_cpf-lcm INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

lcm_add_library(lcmtypes_cpf-lcm-cpp CPP ${cpp_install_headers})
target_include_directories(lcmtypes_cpf-lcm-cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

lcm_install_headers(
  ${CMAKE_CURRENT_BINARY_DIR}/cpf_lcmtypes_export.h
  ${c_install_headers}
  ${cpp_install_headers}
  DESTINATION include/lcmtypes
)

install(TARGETS lcmtypes_cpf-lcm lcmtypes_cpf-lcm-cpp
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
  if(NOT TARGET lcm-java)
    message(STATUS "lcm-java not found: not building Java LCM type bindings")
  else()
    add_jar(lcmtypes_cpf-lcm-jar
      OUTPUT_NAME lcmtypes_cpf-lcm
      INCLUDE_JARS lcm-java
      SOURCES ${java_sources}
    )

    install_jar(lcmtypes_cpf-lcm-jar share/java)

    set(CPF_LCM_INCLUDE_JAVA
      "include(\${CMAKE_CURRENT_LIST_DIR}/cpf-lcmJavaTargets.cmake)"
    )
    export_jars(
      FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}JavaTargets.cmake
      TARGETS lcmtypes_cpf-lcm-jar
    )
    install_jar_exports(
      FILE ${PROJECT_NAME}JavaTargets.cmake
      DESTINATION ${CONFIG_INSTALL_DIR}
      TARGETS lcmtypes_cpf-lcm-jar
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
  TARGETS lcmtypes_cpf-lcm
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
