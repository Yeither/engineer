cmake_minimum_required(VERSION 3.22)
project(exchange)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(OpenCV_INCLUDE_DIRS)
include_directories(${EIGEN3_INCLUDE_DIRS})

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()

add_executable(exchange main.cpp
                GxCamera/GxCamera.cpp
                GxCamera/GxCamera.h Serial.cpp Serial.h AngleSolver.cpp AngleSolver.h station.cpp station.h SimpleProfile.h)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        -lpthread
        -lm
        yaml-cpp
        -lgxiapi)

