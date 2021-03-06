cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(SET CMP0015 NEW)
project(EMPTY_ROOM)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} . ../GAPS/)
link_directories(${PCL_LIBRARY_DIRS} ../GAPS/R3Graphics/ ../GAPS/R3Shapes/
    ../GAPS/RNBasics/ ../GAPS/R2Shapes/
    ../GAPS/jpeg/)
add_definitions(${PCL_DEFINITIONS})
add_library(util util.h util.cpp)
add_library(loopclosure roommodel.h roommodel.cpp)
add_library(pairwise pairwise.cpp pairwise.h plane2plane.h plane2plane.cpp findplanes.cpp
    findplanes.h planecorrespondences.cpp)
add_executable(globalreg globalreg.cpp)
add_executable(planefinder planefinder.cpp)
add_executable(align align.cpp)
add_executable(postprocess postprocess.cpp)
target_link_libraries(globalreg ${PCL_LIBRARIES} R3Graphics.a RNBasics.a
    R3Shapes.a R2Shapes.a jpeg.a GL glut GLU GLEW pairwise util loopclosure)
target_link_libraries(planefinder ${PCL_LIBRARIES} R3Graphics.a RNBasics.a
    R3Shapes.a R2Shapes.a jpeg.a GL glut GLU GLEW pairwise util)
target_link_libraries(align ${PCL_LIBRARIES} R3Graphics.a RNBasics.a
    R3Shapes.a R2Shapes.a jpeg.a GL glut GLU GLEW pairwise util)
target_link_libraries(postprocess ${PCL_LIBRARIES} util)
set(CMAKE_CXX_FLAGS "-Wno-deprecated -std=c++0x")
