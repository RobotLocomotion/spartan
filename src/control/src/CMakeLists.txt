
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake/modules")

# find packages that don't work with normal find_package
find_package(Drake MODULE REQUIRED)

  include_directories(SYSTEM
    ${DRAKE_INCLUDE_DIRS}
  )



#message("DRAKE_INCLUDE_DIRS = ${DRAKE_INCLUDE_DIRS}")


add_executable(plan_runner plan_runner.cc)
target_link_libraries(plan_runner
    ${DRAKE_LIBRARIES}
    ${DRAKE_EXTERNAL_LIBRARIES}
    )
install(TARGETS plan_runner
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

