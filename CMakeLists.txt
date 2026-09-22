cmake_minimum_required(VERSION 3.25)
project(fpvsim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(FPVSIM_WERROR "Treat compiler warnings as errors" OFF)
option(FPVSIM_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)

include(cmake/dependencies.cmake)
include(cmake/fpvsim_target.cmake)

enable_testing()

add_subdirectory(common)
add_subdirectory(simcore)
