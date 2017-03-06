file(TO_CMAKE_PATH "${CMAKE_INSTALL_PREFIX}" CPF_LCM_INSTALL_PREFIX)

configure_file(lcmtypes_cpf-lcm.pc.in
  ${cpf-lcm_BINARY_DIR}/lcmtypes_cpf-lcm.pc @ONLY)
install(FILES ${cpf-lcm_BINARY_DIR}/lcmtypes_cpf-lcm.pc
  DESTINATION lib${LIB_SUFFIX}/pkgconfig)

if(TARGET lcmtypes_cpf-lcm)
  configure_file(lcmtypes_cpf-lcm-java.pc.in
    ${cpf-lcm_BINARY_DIR}/lcmtypes_cpf-lcm-java.pc @ONLY)
  install(FILES ${cpf-lcm_BINARY_DIR}/lcmtypes_cpf-lcm-java.pc
    DESTINATION lib${LIB_SUFFIX}/pkgconfig)
endif()
