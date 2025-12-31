cmake_minimum_required(VERSION 3.10)
project(server LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Paths
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../include)
# set(INCLUDE_COMMON_DIR ${PROJECT_SOURCE_DIR}/../../include_common)
set(LIBRARY_DIR ${PROJECT_SOURCE_DIR}/../library)
set(SOURCE_DIR ${PROJECT_SOURCE_DIR})

# Include headers
include_directories(${INCLUDE_DIR})

# Create executable
add_executable(server ${SOURCE_DIR}/main.cpp)

find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

# link thread
target_link_libraries(server PUBLIC rt Threads::Threads ${JSONCPP_LIBRARIES})