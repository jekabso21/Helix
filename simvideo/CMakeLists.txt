pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)

add_library(fpvsim_simvideo STATIC
    src/config.cpp
    src/control.cpp
    src/pipeline.cpp
    src/runner.cpp
    src/status.cpp)
add_library(fpvsim::simvideo ALIAS fpvsim_simvideo)
target_include_directories(fpvsim_simvideo PUBLIC include)
target_link_libraries(fpvsim_simvideo
    PUBLIC fpvsim::common nlohmann_json::nlohmann_json PkgConfig::GSTREAMER
    PRIVATE fpvsim::options)

add_executable(simvideo src/main.cpp)
target_link_libraries(simvideo PRIVATE fpvsim::simvideo fpvsim::options)

fpvsim_add_test(simvideo_tests
    tests/control_test.cpp
    tests/pipeline_test.cpp
    tests/schedule_test.cpp
    tests/status_test.cpp
    tests/config_test.cpp
    tests/runner_test.cpp
    tests/rtsp_test.cpp
    LIBS fpvsim::simvideo)
