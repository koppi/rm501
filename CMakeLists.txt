cmake_minimum_required(VERSION 3.17)
project(rm501 VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SDL2 CONFIG REQUIRED)

add_executable(rm501 rm501.c)
#target_link_libraries(Vivaria PRIVATE SDL2::SDL2)
target_link_libraries(rm501 PRIVATE SDL2::SDL2 SDL2::SDL2main)
