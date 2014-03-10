cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(SET CMP0015 NEW)
project(EMPTY_ROOM)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} .)
link_directories(${PCL_LIBRARY_DIRS} R3Graphics/ R3Shapes/ RNBasics/ R2Shapes/
    jpeg/)
add_definitions(${PCL_DEFINITIONS})
add_executable(invrender invrender.cpp colorhelper.h colorhelper.cpp)
target_link_libraries(invrender ${PCL_LIBRARIES} R3Graphics.a RNBasics.a
    R3Shapes.a R2Shapes.a jpeg.a GLU)
