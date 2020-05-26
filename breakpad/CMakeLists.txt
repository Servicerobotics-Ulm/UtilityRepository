CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

FIND_FILE(SMART_MACROS SmartMacros2.cmake PATHS $ENV{SMART_ROOT_ACE}/CMakeMacros /opt/smartSoftAce/CMakeMacros)
INCLUDE(${SMART_MACROS})

#FIND_FILE(SMART_OS_CONFIG SmartOSConfig.cmake PATHS $ENV{SMART_ROOT_ACE} /opt/smartSoftAce)
#INCLUDE(${SMART_OS_CONFIG})


SMART_UTILITY_PROJECT(breakpad 1.0.0)

include(ExternalProject)

ExternalProject_Add(breakpad
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/breakpad/src
    DOWNLOAD_COMMAND ""
    INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
    CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/breakpad/src/configure --prefix=<INSTALL_DIR>
    BUILD_IN_SOURCE 1
    BUILD_COMMAND ${CMAKE_BUILD_COMMAND}
    )

ExternalProject_Add_Step(breakpad fetch_breakpad DEPENDERS download # Names of project and custom step
   COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/downloadBreakpad.sh
   COMMENT "Fetching uilities and breakpad sources ...")


IF(AUTOINSTALL)
  ADD_CUSTOM_TARGET(${PROJECT_NAME}AutoInstall ALL COMMAND ${CMAKE_COMMAND} ARGS -P cmake_install.cmake WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMENT "Install ${PROJECT_NAME}")
  ADD_DEPENDENCIES(${PROJECT_NAME}AutoInstall breakpad)
ENDIF(AUTOINSTALL)


INSTALL(FILES ${PROJECT_BINARY_DIR}/breakpadConfig.cmake DESTINATION modules)
INSTALL(FILES ${PROJECT_BINARY_DIR}/breakpadConfigVersion.cmake DESTINATION modules)
INSTALL(FILES ${PROJECT_SOURCE_DIR}/smartComponentBreakpadCallback.hh DESTINATION include/breakpad)
