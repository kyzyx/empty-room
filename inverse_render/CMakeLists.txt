cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(SET CMP0015 NEW)
project(EMPTY_ROOM)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
if(APPLE)
    set(GLLIBS ${OPENGL_LIBRARIES})
else()
    find_package(GLEW REQUIRED)
    include_directories(${GLEW_INCLUDE_DIRS})
    set(GLLIBS ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})
endif()

find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54.0 COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
include_directories(. ../GAPS/)
set(GAPS_LIBRARIES
    R3Graphics.a R3Shapes.a R2Shapes.a RNBasics.a -ljpeg)
link_directories(${PCL_LIBRARY_DIRS}
    ../GAPS/R3Graphics/ ../GAPS/R3Shapes/
    ../GAPS/RNBasics/ ../GAPS/R2Shapes/)

# ----------------------------------------------------------------------------
# Common libraries
add_library(datamanager 
    datamanager/camparams.h datamanager/material.h
    datamanager/rgbe.h datamanager/rgbe.cpp
    datamanager/meshmanager.cpp datamanager/meshmanager.h
    datamanager/meshserver.cpp datamanager/meshserver.h
    datamanager/imagemanager.cpp datamanager/imagemanager.h
    datamanager/imageserver.cpp datamanager/imageserver.h
    datamanager/imageio.cpp datamanager/imageio.h)
target_include_directories(datamanager PRIVATE ${PNG_INCLUDE_DIRS})
target_link_libraries(datamanager ${PCL_LIBRARIES} ${Boost_LIBRARIES}
    ${GAPS_LIBRARIES} ${GLLIBS} ${PNG_LIBRARIES} IlmImf Half rt)

add_executable(dataserver datamanager/dataserver.cpp)
target_link_libraries(dataserver ${PCL_LIBRARIES} datamanager)

add_library(roommodel 
    roommodel/roommodel.h roommodel/RoomModel.cpp roommodel/RoomModel_Internal.h
    roommodel/geometrygenerator.h roommodel/geometrygenerator.cpp
    roommodel/rectanglerenderer.h roommodel/rectanglerenderer.cpp)
target_link_libraries(roommodel datamanager
    ${GAPS_LIBRARIES} ${GLLIBS})
add_library(rendering
    rendering/opengl_compat.h rendering/opengl_compat.cpp
    rendering/loadshader.h rendering/loadshader.cpp
    rendering/rendermanager.h rendering/rendermanager.cpp
    rendering/hemicuberenderer.h rendering/hemicuberenderer.cpp)
target_link_libraries(rendering datamanager roommodel
    ${GAPS_LIBRARIES} ${GLLIBS})
add_library(wallfinder
    wallfinder/orientation_finder.h wallfinder/orientation_finder.cpp
    wallfinder/wall_finder.h wallfinder/wall_finder.cpp
    wallfinder/shortestpath.h wallfinder/shortestpath.cpp)
target_link_libraries(wallfinder datamanager roommodel
    ${GAPS_LIBRARIES} ${GLLIBS})
# ----------------------------------------------------------------------------
# Shared memory applications
add_executable(reprojectapp apps/reprojectapp.cpp
    apps/invrenderapp.cpp apps/invrenderapp.h
    reproject.cpp reproject.h clusterlights.cpp clusterlights.h)
target_link_libraries(reprojectapp ${PCL_LIBRARIES} datamanager)
add_executable(labelimageapp apps/labelimageapp.cpp
    apps/invrenderapp.cpp apps/invrenderapp.h)
target_link_libraries(labelimageapp ${PCL_LIBRARIES} ${GLUT_LIBRARY} datamanager rendering)
add_executable(edgeimageapp apps/edgeimageapp.cpp
    apps/invrenderapp.cpp apps/invrenderapp.h
    orientededgefilter.cpp orientededgefilter.h)
target_link_libraries(edgeimageapp ${PCL_LIBRARIES} ${OpenCV_LIBS} datamanager)
add_executable(linefindapp apps/linefindapp.cpp
    apps/invrenderapp.cpp apps/invrenderapp.h
    linefinder.cpp linefinder.h
    orientededgefilter.cpp orientededgefilter.h)
target_link_libraries(linefindapp ${PCL_LIBRARIES} ${OpenCV_LIBS} datamanager
    roommodel wallfinder)
add_executable(wallfindapp apps/wallfindapp.cpp
    apps/invrenderapp.cpp apps/invrenderapp.h)
target_link_libraries(wallfindapp ${PCL_LIBRARIES} datamanager roommodel
    wallfinder)
# ----------------------------------------------------------------------------
add_executable(invrender EXCLUDE_FROM_ALL monolithic/invrender.cpp
    reproject.h reproject.cpp
    clusterlights.cpp clusterlights.h
    orientededgefilter.cpp orientededgefilter.h
    linefinder.cpp linefinder.h 
    Simplify.h eigen_nnls.h
    rerender.cpp rerender.h
    solver.cpp solvetexture.cpp solver.h
    monolithic/display.cpp monolithic/display.h
    monolithic/parse_args.cpp monolithic/parse_args.h)
target_link_libraries(invrender ${PCL_LIBRARIES} ${OpenCV_LIBS}
    ${GAPS_LIBRARIES} ${GLLIBS} ${GLUT_LIBRARY} IlmImf Half datamanager rendering
    wallfinder)
set(CMAKE_CXX_FLAGS "-Wno-deprecated -std=c++0x")
