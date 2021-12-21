cmake_minimum_required(VERSION 3.12)

project(testmotor)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)


# find aris
if(UNIX)
	add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris/aris-1.5.0)
endif(UNIX)
IF(WIN32)
        find_package(aris REQUIRED PATHS C:/aris/aris-1.5.0)
endif(WIN32)
include_directories(${aris_INCLUDE_DIRS})


include_directories("include")
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(testmotor ${SOURCES})

# add_executable(testmotor src/main.cpp src/robot.cpp src/robot.h src/plan.cpp src/plan.h )

if(UNIX)
    target_link_libraries(testmotor ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(lesson ${aris_LIBRARIES})
endif(WIN32)

