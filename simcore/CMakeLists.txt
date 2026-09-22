configure_file(src/app/version.hpp.in generated/fpvsim/version.hpp @ONLY)

add_library(fpvsim_simcore_models STATIC
    src/env/atmosphere.cpp
    src/physics/aero.cpp
    src/physics/contact.cpp
    src/physics/motor.cpp
    src/physics/propulsion.cpp
    src/physics/rigid_body.cpp
    src/sensors/imu.cpp)
add_library(fpvsim::simcore_models ALIAS fpvsim_simcore_models)
target_include_directories(fpvsim_simcore_models PUBLIC include)
target_link_libraries(fpvsim_simcore_models PUBLIC fpvsim::common PRIVATE fpvsim::options)

add_library(fpvsim_simcore STATIC
    src/api/protocol.cpp
    src/bridge/betaflight.cpp
    src/config/session.cpp
    src/input/control.cpp
    src/input/joystick.cpp
    src/input/mapping.cpp
    src/input/mapping_json.cpp
    src/io/io_thread.cpp
    src/log/csv_log.cpp
    src/net/udp_socket.cpp
    src/pilot/altitude_hold.cpp
    src/sim/loop.cpp
    src/sim/vehicle.cpp)
add_library(fpvsim::simcore ALIAS fpvsim_simcore)
target_include_directories(fpvsim_simcore PUBLIC include src)
target_link_libraries(fpvsim_simcore
    PUBLIC fpvsim::simcore_models nlohmann_json::nlohmann_json
    PRIVATE fpvsim::options PkgConfig::SDL2)

add_executable(simcore src/app/main.cpp)
target_include_directories(simcore PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(simcore PRIVATE
    fpvsim::options
    fpvsim::simcore
    CLI11::CLI11
    spdlog::spdlog)

add_subdirectory(tests)
