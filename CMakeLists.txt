cmake_minimum_required(VERSION 3.24)
project(rm501 LANGUAGES C)

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)

# Increase WASM memory allocation for safety
if(EMSCRIPTEN)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s ALLOW_MEMORY_GROWTH=1 -s INITIAL_MEMORY=512MB")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s MAXIMUM_MEMORY=2GB")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s SAFE_HEAP=1")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s ABORTING_MALLOC=1")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s MAIN_MODULE=2")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s USE_SDL=2")
endif()

find_package(SDL2 QUIET)
if (${SDL2_FOUND})
  MESSAGE(STATUS "Found SDL2.")
else (${SDL2_FOUND})
  MESSAGE(STATUS "Could not find SDL2.")
  FetchContent_Declare(
    SDL2
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-2.32.8
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
  )
  set(SDL_STATIC_PIC ON)
  set(SDL_STATIC ON)
  set(SDL_TEST OFF)
  set(SDL_TESTS OFF)
  FetchContent_MakeAvailable(SDL2)
endif(${SDL2_FOUND})

find_package(SDL2_image QUIET)
if (${SDL2_image_FOUND})
  MESSAGE(STATUS "Found SDL2_image.")
else (${SDL2_image_FOUND})
  MESSAGE(STATUS "Could not find SDL2_image.")
  FetchContent_Declare(
    SDL2_image
    GIT_REPOSITORY https://github.com/libsdl-org/SDL_image.git
    GIT_TAG release-2.8.8
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
  )
  set(SDL2IMAGE_INSTALL OFF)
  set(BUILD_SHARED_LIBS FALSE)
  FetchContent_MakeAvailable(SDL2_image)
endif(${SDL2_image_FOUND})

find_package(SDL2_ttf QUIET)
if (${SDL2_ttf_FOUND})
  MESSAGE(STATUS "Found SDL2_ttf.")
else (${SDL2_ttf_FOUND})
  MESSAGE(STATUS "Could not find SDL2_ttf.")
  set(SDL2TTF_FREETYPE OFF)
  set(SDL2TTF_VENDORED TRUE)
  if(CMAKE_SYSTEM_NAME MATCHES "Emscripten")
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  endif()
  FetchContent_Declare(
    SDL2_ttf
    GIT_REPOSITORY https://github.com/libsdl-org/SDL_ttf.git
    GIT_TAG release-2.24.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
  )
  FetchContent_MakeAvailable(SDL2_ttf)
endif(${SDL2_ttf_FOUND})

add_definitions(-DHAVE_SDL -DHAVE_TRAJGEN)
set(SOURCES rm501.c trajgen.c)

add_executable(rm501 ${SOURCES})

find_package(OpenGL REQUIRED COMPONENTS OpenGL)

if (NOT ${OPENGL_FOUND})
   message(FATAL_ERROR "OpenGL not found")
endif()

if (NOT TARGET OpenGL::GLU)
    message(FATAL_ERROR "GLU not found")
endif(NOT TARGET OpenGL::GLU)

if(CMAKE_SYSTEM_NAME MATCHES "Emscripten")
  target_link_libraries(rm501
    SDL2::SDL2main
    SDL2
    SDL2_image
    SDL2_ttf
    OpenGL::GL
    OpenGL::GLU
    m
  )

  # Include the build directory for font header
  target_include_directories(rm501 PRIVATE ${CMAKE_BINARY_DIR})

  # Enable asyncify for emscripten_sleep support
  set(CMAKE_EXECUTABLE_SUFFIX ".js")
  set(CMAKE_C_FLAGS "-s USE_SDL=2")
  
  set_target_properties(rm501 PROPERTIES
    SUFFIX ".js"
    LINK_FLAGS "-fPIC -s ASSERTIONS=1 -s WASM=1 -s USE_WEBGL2=1 -s EXPORTED_RUNTIME_METHODS=['ccall','cwrap','JSEvents','Browser','HEAPU8','addFunction','removeFunction'] -s EXPORTED_FUNCTIONS=['_main','_malloc','_free'] -s EXPORT_ALL=0 -s FORCE_FILESYSTEM=0 -s NO_EXIT_RUNTIME=1 -s NO_DISABLE_EXCEPTION_CATCHING=1 -s ALLOW_MEMORY_GROWTH=1 -s STACK_SIZE=32MB -s INITIAL_MEMORY=512MB -s MAXIMUM_MEMORY=2GB -s SAFE_HEAP=1 -s ERROR_ON_UNDEFINED_SYMBOLS=0 -s DISABLE_EXCEPTION_CATCHING=0 -s SUPPORT_LONGJMP=1 -O2"
  )
  
  set(CMAKE_DISABLE_FIND_PACKAGE_HarfBuzz TRUE)
  file(COPY "index.html" DESTINATION ${CMAKE_BINARY_DIR})
  
  add_custom_command(TARGET rm501 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_SOURCE_DIR}/index.html
    ${CMAKE_BINARY_DIR}/index.html
    COMMENT "Copying index.html to build directory"
  )
  
  add_custom_command(TARGET rm501 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_SOURCE_DIR}/favicon.ico
    ${CMAKE_BINARY_DIR}/favicon.ico
    COMMENT "Copying favicon.ico to build directory"
  )
else()
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
endif()

if(WIN32)
    add_custom_command(TARGET rm501 POST_BUILD
        COMMAND cmd /c "for /r \"${CMAKE_BINARY_DIR}/_deps\" %i in (*.dll) do copy \"%i\" \"$<TARGET_FILE_DIR:rm501>\""
        COMMENT "Copying DLLs to binary directory"
    )
endif()
