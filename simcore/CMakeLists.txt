configure_file(src/app/version.hpp.in generated/fpvsim/version.hpp @ONLY)

add_executable(simcore src/app/main.cpp)
target_include_directories(simcore PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(simcore PRIVATE
    fpvsim::options
    fpvsim::common
    CLI11::CLI11
    spdlog::spdlog
    nlohmann_json::nlohmann_json)
