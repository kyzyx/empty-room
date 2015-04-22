cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(SET CMP0015 NEW)
project(EMPTY_ROOM)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost 1.54.0 COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS} . ../GAPS/)
link_directories(${PCL_LIBRARY_DIRS} ../GAPS/R3Graphics/ ../GAPS/R3Shapes/
    ../GAPS/RNBasics/ ../GAPS/R2Shapes/
    ../GAPS/jpeg/)
add_definitions(${PCL_DEFINITIONS})
add_library(memory 
    camparams.h
    rgbe.h rgbe.cpp
    meshmanager.cpp meshmanager.h
    meshserver.cpp meshserver.h
    imagemanager.cpp imagemanager.h
    imageserver.cpp imageserver.h
    imageio.cpp imageio.h)
target_link_libraries(memory ${PCL_LIBRARIES} ${Boost_LIBRARIES}
    R3Graphics.a R3Shapes.a R2Shapes.a RNBasics.a jpeg.a
    GL GLU IlmImf Half rt)
add_library(rendering
    roommodel.h RoomModel.cpp RoomModel_Internal.h
    loadshader.h loadshader.cpp
    geometrygenerator.h geometrygenerator.cpp
    rectanglerenderer.h rectanglerenderer.cpp
    rendermanager.h rendermanager.cpp
    hemicuberenderer.h hemicuberenderer.cpp)
target_link_libraries(rendering memory
    R3Graphics.a R3Shapes.a R2Shapes.a RNBasics.a jpeg.a
    GL GLU)

add_executable(dataserver dataserver.cpp)
target_link_libraries(dataserver ${PCL_LIBRARIES} memory)
add_executable(reprojectapp reprojectapp.cpp invrenderapp.cpp invrenderapp.h
    reproject.cpp reproject.h clusterlights.cpp clusterlights.h)
target_link_libraries(reprojectapp ${PCL_LIBRARIES} memory)
add_executable(wallfindapp wallfindapp.cpp invrenderapp.cpp invrenderapp.h
    RoomModel_Internal.h roommodel.h RoomModel.cpp
    orientation_finder.h orientation_finder.cpp wall_finder.h wall_finder.cpp
    shortestpath.h shortestpath.cpp)
target_link_libraries(wallfindapp ${PCL_LIBRARIES} memory)

add_executable(invrender invrender.cpp
    reproject.h reproject.cpp
    wall_finder.h wall_finder.cpp material.h
    hemicuberenderer.h hemicuberenderer.cpp
    orientation_finder.cpp orientation_finder.h
    shortestpath.cpp shortestpath.h
    Simplify.h eigen_nnls.h
    solver.cpp solvetexture.cpp solver.h clusterlights.cpp clusterlights.h
    RoomModel.cpp roommodel.h RoomModel_Internal.h
    display.cpp display.h parse_args.cpp parse_args.h rerender.cpp rerender.h)
target_link_libraries(invrender ${PCL_LIBRARIES} ${OpenCV_LIBS}
    R3Graphics.a R3Shapes.a R2Shapes.a RNBasics.a
    jpeg.a GL glut GLU GLEW IlmImf Half memory rendering)
set(CMAKE_CXX_FLAGS "-Wno-deprecated -std=c++0x")
