cmake_minimum_required(VERSION 3.10.0)
project(MultiCamPylonSample VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -w -pthread -Wno-deprecated-declarations -fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

##################### Export external libraries #####################
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(OpenCV REQUIRED)

#Pylon
find_package(Pylon REQUIRED)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/FindPylon.cmake")
endif()
message(STATUS ${Pylon_INCLUDE_DIRS})
include_directories(${Pylon_INCLUDE_DIRS})

add_subdirectory(SignalGeneratorLib)

add_executable(multi_pylon_test main.cpp)
target_link_libraries(multi_pylon_test PUBLIC ${Pylon_LIBRARIES} ${OpenCV_LIBS} SignalGeneratorLib)
target_include_directories(multi_pylon_test PUBLIC SignalGeneratorLib)