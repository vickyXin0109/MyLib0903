cmake_minimum_required(VERSION 3.17)
project(add_lib_static)

include_directories(include)
add_library(MyLib STATIC /${PROJECT_SOURCE_DIR}/src/MyLib.cpp)

link_directories(${PROJECT_BINARY_DIR})
add_executable(Example temp_example.cpp)
target_link_libraries(Example MyLib)