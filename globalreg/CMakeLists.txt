cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(SET CMP0015 NEW)
project(EMPTY_ROOM)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} . ../GAPS/)
link_directories(${PCL_LIBRARY_DIRS} ../GAPS/R3Graphics/ ../GAPS/R3Shapes/
    ../GAPS/RNBasics/ ../GAPS/R2Shapes/
    ../GAPS/jpeg/)
add_definitions(${PCL_DEFINITIONS})
add_executable(globalreg globalreg.cpp pairwise.cpp pairwise.h findplanes.cpp findplanes.h)
add_executable(planefinder planefinder.cpp pairwise.cpp pairwise.h findplanes.cpp findplanes.h)
target_link_libraries(globalreg ${PCL_LIBRARIES} R3Graphics.a RNBasics.a
    R3Shapes.a R2Shapes.a jpeg.a GL glut GLU GLEW)
target_link_libraries(planefinder ${PCL_LIBRARIES} R3Graphics.a RNBasics.a
    R3Shapes.a R2Shapes.a jpeg.a GL glut GLU GLEW)
set(CMAKE_CXX_FLAGS "-Wno-deprecated -std=c++0x")
