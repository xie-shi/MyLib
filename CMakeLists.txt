cmake_minimum_required(VERSION 3.10)
project(DataTest)

message(=============================================================)
message("-- Ready to compile ")
message(=============================================================)
find_package(OpenCV REQUIRED)
message(-----------------------------------------)
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})
message(-----------------------------------------)


find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

message(-----------------------------------------)
message(STATUS "    version: ${PCL_VERSION}")
message(STATUS "    libraries: ${PCL_LIBRARIES}")
message(STATUS "    include path: ${PCL_INCLUDE_DIRS}")
message(-----------------------------------------)

set(CMAKE_CXX_STANDARD 14)
add_executable(DataTest main.cpp)

target_link_libraries(DataTest  ${OpenCV_LIBS} ${PCL_LIBRARIES})