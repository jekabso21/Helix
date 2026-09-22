add_library(fpvsim_common INTERFACE)
add_library(fpvsim::common ALIAS fpvsim_common)
target_include_directories(fpvsim_common INTERFACE include)
target_link_libraries(fpvsim_common INTERFACE fpvsim::eigen)

fpvsim_add_test(common_tests
    tests/sim_time_test.cpp
    tests/frames_test.cpp
    tests/spsc_queue_test.cpp
    LIBS fpvsim::common)
