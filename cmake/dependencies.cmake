include(FetchContent)

# SOURCE_SUBDIR names a missing directory to skip Eigen's slow CMake project
FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR no_cmake
    SYSTEM)

FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.12.0
    GIT_SHALLOW TRUE
    SYSTEM)

FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.15.3
    GIT_SHALLOW TRUE
    SYSTEM)

FetchContent_Declare(CLI11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG v2.5.0
    GIT_SHALLOW TRUE
    SYSTEM)

FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
    GIT_SHALLOW TRUE
    SYSTEM)

set(JSON_BuildTests OFF CACHE INTERNAL "")
set(CLI11_BUILD_TESTS OFF CACHE INTERNAL "")
set(CLI11_BUILD_EXAMPLES OFF CACHE INTERNAL "")
set(BUILD_GMOCK OFF CACHE INTERNAL "")
set(INSTALL_GTEST OFF CACHE INTERNAL "")

FetchContent_MakeAvailable(eigen nlohmann_json spdlog CLI11 googletest)
include(GoogleTest)

add_library(fpvsim_eigen INTERFACE)
add_library(fpvsim::eigen ALIAS fpvsim_eigen)
target_include_directories(fpvsim_eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
