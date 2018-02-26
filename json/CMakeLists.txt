CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

FIND_FILE(SMART_MACROS SmartMacros.cmake PATHS $ENV{SMART_ROOT_ACE} /opt/smartSoftAce)
INCLUDE(${SMART_MACROS})

SMART_UTILITY_PROJECT(Json)

SMART_VERSION(7.6.0)

# SMART_UTILITY_BUILD_DEBIAN_PACKAGE(${PROJECT_NAME} ${PACKAGE_VERSION_STRING})

INCLUDE(ExternalProject)

ExternalProject_Add(JsonPackage
	PREFIX json-tmp
	#DOWNLOAD_DIR json-download
	# URL "http://sourceforge.net/projects/libjson/files/libjson_7.6.0.zip/download"
	URL "http://downloads.sourceforge.net/project/libjson/libjson_7.6.0.zip"
	URL_MD5 "dcb326038bd9b710b8f717580c647833" # copy the hash from sourceforge site by hand (unfortunatelly sf does not provide a link for this)
	PATCH_COMMAND patch ${PROJECT_BINARY_DIR}/json-src/JSONOptions.h < ${PROJECT_SOURCE_DIR}/JSONOptions.h.patch
	SOURCE_DIR json-src
	CONFIGURE_COMMAND ""
	BINARY_DIR json-src
	BUILD_COMMAND ${CMAKE_BUILD_TOOL}
	INSTALL_COMMAND ${CMAKE_BUILD_TOOL} install prefix=${PROJECT_BINARY_DIR}/json-install libdir=libjson.a
)

IF(AUTOINSTALL)
  ADD_CUSTOM_TARGET(${PROJECT_NAME}AutoInstall ALL COMMAND ${CMAKE_COMMAND} ARGS -P cmake_install.cmake WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMENT "Install ${PROJECT_NAME}")
  ADD_DEPENDENCIES(${PROJECT_NAME}AutoInstall JsonPackage)
ENDIF(AUTOINSTALL)

SET_SOURCE_FILES_PROPERTIES(${PROJECT_BINARY_DIR}/json-install/libjson.a PROPERTIES GENERATED TRUE)

INSTALL(FILES ${PROJECT_BINARY_DIR}/json-install/libjson.a DESTINATION lib)
INSTALL(DIRECTORY ${PROJECT_BINARY_DIR}/json-install/include/libjson DESTINATION include)
INSTALL(FILES ${PROJECT_BINARY_DIR}/JsonConfig.cmake DESTINATION modules)

