cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(SET CMP0015 NEW)
project(EMPTY_ROOM)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} . ../GAPS/)
link_directories(${PCL_LIBRARY_DIRS} ../GAPS/R3Graphics/ ../GAPS/R3Shapes/
    ../GAPS/RNBasics/ ../GAPS/R2Shapes/
    ../GAPS/jpeg/)
add_definitions(${PCL_DEFINITIONS})
add_executable(invrender rgbe.cpp rgbe.h invrender.cpp colorhelper.h colorhelper.cpp reproject.h
    reproject.cpp mesh.h mesh.cpp wall_finder.cpp wall_finder.h
    orientation_finder.cpp orientation_finder.h
    solver.cpp solver.h clusterlights.cpp clusterlights.h
    display.cpp display.h parse_args.cpp parse_args.h rerender.cpp rerender.h)
target_link_libraries(invrender ${PCL_LIBRARIES} R3Graphics.a RNBasics.a
    R3Shapes.a R2Shapes.a jpeg.a GL glut GLU GLEW)
set(CMAKE_CXX_FLAGS "-Wno-deprecated -std=c++0x")
