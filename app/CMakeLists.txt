add_executable(shell-app main.cpp PathPlanner.cpp Map.cpp OpenList.cpp Robot.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
