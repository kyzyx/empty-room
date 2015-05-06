add_library(datamanager 
    camparams.h material.h
    rgbe.h rgbe.cpp
    meshmanager.cpp meshmanager.h
    meshserver.cpp meshserver.h
    imagemanager.cpp imagemanager.h
    imageserver.cpp imageserver.h
    imageio.cpp imageio.h)
target_link_libraries(datamanager ${PCL_LIBRARIES} ${Boost_LIBRARIES}
    R3Graphics.a R3Shapes.a R2Shapes.a RNBasics.a jpeg.a
    ${GLLIBS} IlmImf Half rt)

add_executable(dataserver dataserver.cpp)
target_link_libraries(dataserver ${PCL_LIBRARIES} datamanager)
set(CMAKE_CXX_FLAGS "-Wno-deprecated -std=c++0x")
