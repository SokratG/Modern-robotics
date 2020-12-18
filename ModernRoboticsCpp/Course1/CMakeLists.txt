cmake_minimum_required (VERSION 3.16)

set(${PROJECT_NAME} "ModernRoboticsCourse1")
project(${PROJECT_NAME})
project(${PROJECT_NAME} LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 11)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("../Eigen")
include_directories("../MRlib")
include_directories("${PROJECT_BINARY_DIR}")


file(GLOB SRCCPPS "src/*.cpp")

LINK_DIRECTORIES(../MRlib/Debug)

# Add source to this project's executable.
add_executable (${PROJECT_NAME} ${SRCCPPS})


# link the new library target with the binary target
TARGET_LINK_LIBRARIES( ${PROJECT_NAME}
    PUBLIC 
        ModernRoboticLib
)