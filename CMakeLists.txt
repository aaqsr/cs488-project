# After reconfiguring any aspect of CMake project: cmake --preset debug
#   Runs configuration: sets up build/debug dir, generates build files, etc.

# Actually building or rebuilding: cmake --build --preset debug

cmake_minimum_required(VERSION 3.19)
project(CS488)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(external/glew)
add_subdirectory(external/glfw)

file(GLOB SRC_FILES
        src/*.cpp)

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
# TODO: THIS DOES NOT WORK ON WINDOWS AAAA
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

# Copy or symlink compile_commands.json to the project root
# (only if it's not already there)
# TODO: THIS DOES NOT WORK ON WINDOWS AAAA
add_custom_command(TARGET CS488 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_BINARY_DIR}/compile_commands.json"
        "${CMAKE_SOURCE_DIR}/compile_commands.json"
)

file(CREATE_LINK "${CMAKE_SOURCE_DIR}/shaders" "${CMAKE_BINARY_DIR}/shaders" SYMBOLIC)
file(CREATE_LINK "${CMAKE_SOURCE_DIR}/assets" "${CMAKE_BINARY_DIR}/assets" SYMBOLIC)
