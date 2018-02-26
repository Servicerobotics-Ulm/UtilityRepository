
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

FIND_FILE(SMART_MACROS SmartMacros2.cmake PATHS $ENV{SMART_ROOT_ACE}/CMakeMacros /opt/smartSoftAce/CMakeMacros)
INCLUDE(${SMART_MACROS})

SMART_UTILITY_PROJECT(Armadillo 1.0.0)

#SMART_UTILITY_BUILD_DEBIAN_PACKAGE(${PROJECT_NAME} ${PACKAGE_VERSION_STRING})

# we create an INTERFACE library that does not need to be compiled but,
# other than that has all the properties of a library target (like the include directories)
ADD_LIBRARY(${PROJECT_NAME} INTERFACE)

# setup include directories for this target
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>  # <prefix>/include/mylib
)

SMART_UTILITY_INSTALL(${PROJECT_NAME})

# add header files for installation into default install folder
SET(HDRS armadillo.hh)
INSTALL(FILES ${HDRS} DESTINATION include/${PROJECT_NAME})

# installation into a custom subfolder
FILE(GLOB ARMA_BITS "armadillo_bits/*.hpp")
INSTALL(FILES ${ARMA_BITS} DESTINATION include/${PROJECT_NAME}/armadillo_bits)



