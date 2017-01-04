cmake_minimum_required(VERSION 3.2.1)
project (scratch)

include(CMakeToolsHelpers OPTIONAL)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -Wpedantic")

add_subdirectory(app)
add_subdirectory(test)
add_subdirectory(vendor/googletest/googletest)
