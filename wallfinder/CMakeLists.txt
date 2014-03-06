cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(EMPTY_ROOM)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(floorplan floorplan.cpp
    orientation_finder.cpp orientation_finder.h
    wall_finder.cpp wall_finder.h)
target_link_libraries(floorplan ${PCL_LIBRARIES})
