CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

FIND_FILE(SMART_MACROS SmartMacros2.cmake PATHS $ENV{SMART_ROOT_ACE}/CMakeMacros /opt/smartSoftAce/CMakeMacros)
INCLUDE(${SMART_MACROS})

SMART_UTILITY_PROJECT(TransformationMatrices 1.0.0)

#SMART_UTILITY_BUILD_DEBIAN_PACKAGE(${PROJECT_NAME} ${PACKAGE_VERSION_STRING})

# we create an INTERFACE library that does not need to be compiled but,
# other than that has all the properties of a library target (like the include directories)
ADD_LIBRARY(${PROJECT_NAME} INTERFACE)

# setup include directories for this target
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>  # <prefix>/include/mylib
)


# add header files for installation into default install folder
SET(HDRS EulerTransformationMatrices.hh)
INSTALL(FILES ${HDRS} DESTINATION include/${PROJECT_NAME})

SMART_UTILITY_INSTALL(${PROJECT_NAME})

