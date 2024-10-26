cmake_minimum_required(VERSION 3.10)
project(MyOpenCVProject)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(my_program src/main.cpp)
target_link_libraries(my_program ${OpenCV_LIBS})
