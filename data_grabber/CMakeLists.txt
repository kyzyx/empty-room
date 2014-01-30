cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(EMPTY_ROOM)
find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(data_grabber data_grabber.cpp)
target_link_libraries(data_grabber ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
    ${PCL_VISUALIZATION_LIBRARIES})
