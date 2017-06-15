file(TO_CMAKE_PATH "${CMAKE_INSTALL_PREFIX}" SPARTAN_LCM_INSTALL_PREFIX)

configure_file(lcmtypes_spartan-lcm.pc.in
  ${spartan-lcm_BINARY_DIR}/lcmtypes_spartan-lcm.pc @ONLY)
install(FILES ${spartan-lcm_BINARY_DIR}/lcmtypes_spartan-lcm.pc
  DESTINATION lib${LIB_SUFFIX}/pkgconfig)

if(TARGET lcmtypes_spartan-lcm)
  configure_file(lcmtypes_spartan-lcm-java.pc.in
    ${spartan-lcm_BINARY_DIR}/lcmtypes_spartan-lcm-java.pc @ONLY)
  install(FILES ${spartan-lcm_BINARY_DIR}/lcmtypes_spartan-lcm-java.pc
    DESTINATION lib${LIB_SUFFIX}/pkgconfig)
endif()
