# After reconfiguring any aspect of CMake project: cmake --preset debug
#   Runs configuration: sets up build/debug dir, generates build files, etc.

# Actually building or rebuilding: cmake --build --preset debug

cmake_minimum_required(VERSION 3.19)
project(CS488)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(external/glew)
add_subdirectory(external/glfw)

file(GLOB SRC_FILES
        src/*.cpp
        src/*/*.cpp)

# Symbolic link for media directory. May not work on windows?
# file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/media"
# "${CMAKE_CURRENT_BINARY_DIR}/media" SYMBOLIC)

add_executable(CS488
        ${SRC_FILES})

# Headers
target_include_directories(CS488
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/external/glfw/include
        ${CMAKE_CURRENT_SOURCE_DIR}/external/glew/include
        ${CMAKE_CURRENT_SOURCE_DIR}/external/linalg
        ${CMAKE_CURRENT_SOURCE_DIR}/external/stb
)

# Link libs
target_link_libraries(CS488 glfw libglew_static)

# Generate compile_commands.json in the build directory
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

# If we generated the compile commands, then copy or symlink compile_commands.json to the project root (only if it's not already there)
if (CMAKE_EXPORT_COMPILE_COMMANDS)
    add_custom_command(TARGET CS488 POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/compile_commands.json"
            "${CMAKE_SOURCE_DIR}/compile_commands.json"
    )
endif ()

if (WIN32)
    # On Windows, copy directories instead of creating symbolic links
    add_custom_command(TARGET CS488 POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/shaders"
            "${CMAKE_BINARY_DIR}/shaders"
            COMMENT "copying shaders to build directory"
    )

    add_custom_command(TARGET CS488 POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/assets"
            "${CMAKE_BINARY_DIR}/assets"
            COMMENT "copying assets to build directory"
    )
else ()
    # use symbolic links
    file(CREATE_LINK "${CMAKE_SOURCE_DIR}/shaders" "${CMAKE_BINARY_DIR}/shaders" SYMBOLIC)
    file(CREATE_LINK "${CMAKE_SOURCE_DIR}/assets" "${CMAKE_BINARY_DIR}/assets" SYMBOLIC)
endif ()

# MS BUILD YUM YUM WOO
# TODO: NO CLUE whether this works...
if (WIN32 AND CMAKE_GENERATOR MATCHES "Visual Studio")
    # set startup project for Visual Studio
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT CS488)

    # set working directory for debugging in Visual Studio
    set_target_properties(CS488 PROPERTIES
            VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
endif ()

# platform based compiler flags
if (MSVC)
    # why oh why microsoft are these not just gcc/clang flags sigh
    target_compile_options(CS488 PRIVATE
            /W4  # Warning level 4
            $<$<CONFIG:Debug>:/Od /Zi>  # Debug: no optimization, debug info
            $<$<CONFIG:Release>:/O2>    # Release: optimize for speed
    )

    # define DEBUG for debug builds
    target_compile_definitions(CS488 PRIVATE
            $<$<CONFIG:Debug>:DEBUG>
            $<$<CONFIG:Release>:NDEBUG>
    )
else ()
    # gcc/clang flags
    target_compile_options(CS488 PRIVATE
            -Wall -Wextra
            $<$<CONFIG:Debug>:-g -O0>
            $<$<CONFIG:Release>:-O3>
    )

    # define DEBUG for debug builds
    target_compile_definitions(CS488 PRIVATE
            $<$<CONFIG:Debug>:DEBUG>
            $<$<CONFIG:Release>:NDEBUG>
    )
endif ()
