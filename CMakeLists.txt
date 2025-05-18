cmake_minimum_required(VERSION 3.17)
project(rm501 LANGUAGES C VERSION 0.0.1)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED YES)
#set(CMAKE_CXX_EXTENSIONS ON)

add_definitions(-DHAVE_SDL)
set(SOURCES rm501.c)

set(FPHSA_NAME_MISMATCHED 1) # Suppress warnings, see https://cmake.org/cmake/help/v3.17/module/FindPackageHandleStandardArgs.html
find_package(SDL2 REQUIRED)
find_package(SDL2_ttf REQUIRED)
find_package(SDL2_image REQUIRED)
unset(FPHSA_NAME_MISMATCHED)

find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(GLUT REQUIRED)

if (NOT ${OPENGL_FOUND})
    message(FATAL_ERROR "OpenGL not found")
endif()

if (NOT TARGET OpenGL::GLU)
    message(FATAL_ERROR "GLU not found")
endif(NOT TARGET OpenGL::GLU)

include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${SDL2_TTF_INCLUDE_DIR} ${SDL2_IMAGE_INCLUDE_DIR})

add_executable(rm501 ${SOURCES})

add_dependencies(rm501 OpenGL::GL)

target_link_libraries(rm501 PRIVATE m SDL2 SDL2::Main ${SDL2_TTF_LIBRARY} ${SDL2_IMAGE_LIBRARY} OpenGL::GL OpenGL::GLU GLUT::GLUT)
add_definitions(-DSDL_MAIN_HANDLED)
