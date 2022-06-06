cmake_minimum_required(VERSION 3.17)
project(rm501 VERSION 0.0.1)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED YES)
#set(CMAKE_CXX_EXTENSIONS ON)

add_definitions(-DHAVE_SDL)
set(SOURCES rm501.c)

find_package(SDL2 CONFIG REQUIRED)
find_package(SDL2TTF REQUIRED)
find_package(SDL2IMAGE REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(GLUT REQUIRED)

if (NOT ${OPENGL_FOUND})
    message(FATAL_ERROR "OpenGL not found")
endif()

if (NOT TARGET OpenGL::GLU)
    message(FATAL_ERROR "GLU not found")
endif(NOT TARGET OpenGL::GLU)

include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${SDL2TTF_INCLUDE_DIR} ${SDL2IMAGE_INCLUDE_DIR})

add_executable(rm501 ${SOURCES})

add_dependencies(rm501 OpenGL::GL)

target_link_libraries(rm501 PRIVATE m SDL2::SDL2 SDL2::SDL2main ${SDL2TTF_LIBRARY} ${SDL2IMAGE_LIBRARY} OpenGL::GL OpenGL::GLU GLUT::GLUT)
add_definitions(-DSDL_MAIN_HANDLED)
