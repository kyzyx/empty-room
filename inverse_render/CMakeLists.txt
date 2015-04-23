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
include_directories(. ../GAPS/)
link_directories(${PCL_LIBRARY_DIRS}
    ../GAPS/R3Graphics/ ../GAPS/R3Shapes/
    ../GAPS/RNBasics/ ../GAPS/R2Shapes/
    ../GAPS/jpeg/)

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
    ${GLLIBS} IlmImf Half rt)
add_library(rendering
    roommodel.h RoomModel.cpp RoomModel_Internal.h
    opengl_compat.h opengl_compat.cpp
    loadshader.h loadshader.cpp
    geometrygenerator.h geometrygenerator.cpp
    rectanglerenderer.h rectanglerenderer.cpp
    rendermanager.h rendermanager.cpp
    hemicuberenderer.h hemicuberenderer.cpp)
target_link_libraries(rendering memory
    R3Graphics.a R3Shapes.a R2Shapes.a RNBasics.a jpeg.a
    ${GLLIBS})

add_executable(dataserver dataserver.cpp)
target_link_libraries(dataserver ${PCL_LIBRARIES} memory)
add_executable(reprojectapp reprojectapp.cpp invrenderapp.cpp invrenderapp.h
    reproject.cpp reproject.h clusterlights.cpp clusterlights.h)
target_link_libraries(reprojectapp ${PCL_LIBRARIES} memory)
add_executable(labelimageapp labelimageapp.cpp invrenderapp.cpp invrenderapp.h
    material.h)
target_link_libraries(labelimageapp ${PCL_LIBRARIES} ${GLUT_LIBRARY} memory rendering)
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
    jpeg.a ${GLLIBS} ${GLUT_LIBRARY} IlmImf Half memory rendering)
set(CMAKE_CXX_FLAGS "-Wno-deprecated -std=c++0x")
