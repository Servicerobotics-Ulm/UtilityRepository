cmake_minimum_required(VERSION 3.0)

FIND_FILE(SMART_MACROS SmartMacros2.cmake PATHS $ENV{SMART_ROOT_ACE}/CMakeMacros /opt/smartSoftAce/CMakeMacros)
INCLUDE(${SMART_MACROS})

SMART_UTILITY_PROJECT (YAML_CPP 0.2.5)

#SMART_UTILITY_BUILD_DEBIAN_PACKAGE("yaml" ${PACKAGE_VERSION_STRING})

set(LIB_TYPE SHARED)

if(IPHONE)
  set(CMAKE_OSX_SYSROOT iphoneos2.2.1)
  set(LIB_TYPE)
endif(IPHONE)

if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS "-O2 -Wall -Wextra -pedantic -Wno-long-long ${CMAKE_CXX_FLAGS}")
endif(CMAKE_COMPILER_IS_GNUCC)

if(MSVC)
  set(LIB_TYPE) # I can't figure out how CMake handles Windows shared libraries
  set(CMAKE_CXX_FLAGS "/W3 /wd4127 /wd4355 /D_SCL_SECURE_NO_WARNINGS ${CMAKE_CXX_FLAGS}")
endif(MSVC)

enable_testing()

option(YAML_CPP_BUILD_TOOLS "Enables or disables testing and parse tools" true)

if(WIN32)
  set(_library_dir bin) # .dll are in PATH, like executables
else(WIN32)
  set(_library_dir lib)
endif(WIN32)

set(INCLUDE_INSTALL_DIR include/YAML_CPP)
set(LIB_INSTALL_DIR ${_library_dir}${LIB_SUFFIX})

#
set(_INSTALL_DESTINATIONS
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION ${LIB_INSTALL_DIR}
	ARCHIVE DESTINATION lib${LIB_SUFFIX}
)
#
file(GLOB public_headers include/[a-z]*.h)
file(GLOB private_headers src/[a-z]*.h)
file(GLOB sources src/[a-z]*.cpp)

#include_directories(${YAML_CPP_SOURCE_DIR}/include)

add_library(${PROJECT_NAME}
	${LIB_TYPE}
	${public_headers}
	${private_headers}
	${sources}
)
set_target_properties(${PROJECT_NAME} PROPERTIES
	VERSION "${YAML_CPP_VERSION}"
	SOVERSION "${YAML_CPP_VERSION_MAJOR}.${YAML_CPP_VERSION_MINOR}"
)

TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>  # <prefix>/include/mylib
)

#install(TARGETS yaml-cpp ${_INSTALL_DESTINATIONS})

SMART_UTILITY_INSTALL(${PROJECT_NAME})
install(
	FILES ${public_headers}
	DESTINATION ${INCLUDE_INSTALL_DIR}
)

if(UNIX)
	set(PC_FILE ${CMAKE_BINARY_DIR}/yaml-cpp.pc)
	configure_file("yaml-cpp.pc.cmake" ${PC_FILE} @ONLY)
	install(FILES ${PC_FILE} DESTINATION ${LIB_INSTALL_DIR}/pkgconfig)
endif(UNIX)

if(YAML_CPP_BUILD_TOOLS)
	add_subdirectory (test)
	add_subdirectory (util)
endif(YAML_CPP_BUILD_TOOLS)
