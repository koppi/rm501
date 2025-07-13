cmake_minimum_required(VERSION 3.24)
project(rm501 LANGUAGES C)

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(
        SDL2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG release-2.32.8
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(SDL2)

FetchContent_Declare(
        SDL2_image
        GIT_REPOSITORY https://github.com/libsdl-org/SDL_image.git
        GIT_TAG release-2.8.8
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
)

# set(SDL2IMAGE_INSTALL OFF)
# set(BUILD_SHARED_LIBS FALSE)

FetchContent_MakeAvailable(SDL2_image)

set(SDL2TTF_VENDORED ON)
FetchContent_Declare(
        SDL2_ttf
        GIT_REPOSITORY https://github.com/libsdl-org/SDL_ttf.git
        GIT_TAG release-2.24.0
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(SDL2_ttf)
set(SDL2TTF_FREETYPE OFF)

add_definitions(-DHAVE_SDL -DHAVE_TRAJGEN)
set(SOURCES rm501.c trajgen.c)
add_executable(rm501 ${SOURCES})

find_package(OpenGL REQUIRED COMPONENTS OpenGL)
#find_package(GLUT REQUIRED)

if (NOT ${OPENGL_FOUND})
   message(FATAL_ERROR "OpenGL not found")
endif()

if (NOT TARGET OpenGL::GLU)
    message(FATAL_ERROR "GLU not found")
endif(NOT TARGET OpenGL::GLU)

target_link_libraries(rm501
        PRIVATE
        SDL2::SDL2main
        SDL2
        SDL2_image
        SDL2_ttf
        OpenGL::GL
        OpenGL::GLU
        m
)
