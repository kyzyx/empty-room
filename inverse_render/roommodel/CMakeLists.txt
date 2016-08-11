add_library(roommodel 
    roommodel.h RoomModel.cpp RoomModel_Internal.h
    geometrygenerator.h geometrygenerator.cpp
    rectanglerenderer.h rectanglerenderer.cpp)
target_link_libraries(roommodel datamanager
    R3Graphics.a R3Shapes.a R2Shapes.a RNBasics.a jpeg.a ${GLLIBS})
set(CMAKE_CXX_FLAGS "-Wno-deprecated -std=c++0x")
