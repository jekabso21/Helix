# Built on demand by scripts/build_extension.sh, not by the normal presets: godot-cpp is large
cmake_minimum_required(VERSION 3.25)
project(fpvsim_extension LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT DEFINED FPVSIM_GODOT_CPP_DIR)
    set(FPVSIM_GODOT_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/godot-cpp")
endif()
if(NOT EXISTS "${FPVSIM_GODOT_CPP_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "godot-cpp missing: git submodule update --init third_party/godot-cpp")
endif()

add_subdirectory("${FPVSIM_GODOT_CPP_DIR}" godot-cpp)

add_library(fpvsim_extension SHARED
    src/frame_publisher.cpp
    src/register_types.cpp)
target_include_directories(fpvsim_extension PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../common/include")
target_link_libraries(fpvsim_extension PRIVATE godot-cpp)
set_target_properties(fpvsim_extension PROPERTIES
    OUTPUT_NAME "fpvsim_extension"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../bin")
