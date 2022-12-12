cmake_minimum_required (VERSION 3.22)

project("optlib" VERSION 0.0.1
  DESCRIPTION "A header-only c++ library that provides algoritms for gradient optimization of single value function."
  HOMEPAGE_URL "https://github.com/borodziejciesla/optlib")

include(GNUInstallDirs)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/io>
)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)

# enable_testing()
# add_subdirectory(test)

# locations are provided by GNUInstallDirs
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file("optlibConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(
  EXPORT ${PROJECT_NAME}_Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION include)

set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")

include(CPack)