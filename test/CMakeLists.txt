cmake_minimum_required(VERSION 3.10)
project(test_plan)
set(CMAKE_CXX_STANDARD 17)


include_directories("../include")
# file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(test_plan "../src/T_planner.cpp" "../scr/plan.cpp")

# add_executable(testmotor src/main.cpp src/robot.cpp src/robot.h src/plan.cpp src/plan.h )



