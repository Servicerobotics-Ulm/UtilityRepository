cmake_minimum_required(VERSION 3.5)

project(example)

# this is the build-tree version
FIND_PACKAGE(nlohman_json PATHS ${PROJECT_SOURCE_DIR}/../build)
# this is the installed version:
#FIND_PACKAGE(nlohman_json PATHS $ENV{SMART_ROOT_ACE}/modules)

add_executable(example main.cpp)
target_link_libraries(example nlohmann_json::nlohmann_json)
