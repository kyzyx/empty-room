cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(EMPTY_ROOM)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(synthetic_preprocess synthetic_preprocess.cpp)
target_link_libraries(synthetic_preprocess ${PCL_LIBRARIES})
set(CMAKE_CXX_FLAGS "-Wno-deprecated -std=c++0x")
