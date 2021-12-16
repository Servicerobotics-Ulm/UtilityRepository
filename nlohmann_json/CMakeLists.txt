cmake_minimum_required(VERSION 3.5)

project(nlohmann_json VERSION 3.7)

include(ExternalProject)

ExternalProject_Add(NlohmannJsonGit
  GIT_REPOSITORY https://github.com/toolify-robotics/json.git
  GIT_PROGRESS TRUE
  CMAKE_ARGS -DJSON_BuildTests=OFF -DCMAKE_INSTALL_PREFIX=${PROJECT_BINARY_DIR}/tmp-install
)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    IF(DEFINED ENV{SMART_ROOT_ACE})
      # in case the environment variable SMART_ROOT_ACE is defined, its content is used as the default install prefix
      SET(CMAKE_INSTALL_PREFIX $ENV{SMART_ROOT_ACE} CACHE PATH "Set default SmartSoft install prefix" FORCE)
    ELSE()
      MESSAGE(WARNING "CMAKE_INSTALL_PREFIX not defined, please either define the environment variable SMART_ROOT_ACE or directly set CMAKE_INSTALL_PREFIX")
    ENDIF()
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

OPTION(AUTOINSTALL "Activates automatic installation of build artefacts" ON)

IF(${AUTOINSTALL})
  # automatically run "make install" after regular "make" for the communication object
  ADD_CUSTOM_TARGET(autoinstall ALL COMMAND ${CMAKE_COMMAND} ARGS -P cmake_install.cmake WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMENT "Install ${PROJECT_NAME}")
  ADD_DEPENDENCIES(autoinstall NlohmannJsonGit)
ENDIF(${AUTOINSTALL})

install(DIRECTORY ${PROJECT_BINARY_DIR}/tmp-install/include/nlohmann DESTINATION include)
install(DIRECTORY ${PROJECT_BINARY_DIR}/tmp-install/lib/cmake/nlohmann_json DESTINATION lib/cmake)
if(WIN32)
install(FILES ${PROJECT_BINARY_DIR}/tmp-install/nlohmann_json.natvis DESTINATION ${CMAKE_INSTALL_PREFIX})
endif(WIN32)

