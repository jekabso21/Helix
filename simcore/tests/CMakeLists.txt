set(bf_target_h
    ${PROJECT_SOURCE_DIR}/third_party/betaflight/src/platform/SIMULATOR/target/SITL/target.h)

if(NOT EXISTS ${bf_target_h})
    message(WARNING "Betaflight submodule not checked out: simcore protocol layout test skipped")
    return()
endif()

# Copy the packet typedefs out of the pinned Betaflight source so the test compares against them
file(READ ${bf_target_h} bf_content)
string(FIND "${bf_content}" "#define SIMULATOR_MAX_RC_CHANNELS" defines_begin)
string(FIND "${bf_content}" "double timestamp;" first_field)
string(FIND "${bf_content}" "} servo_packet_raw;" structs_end)
if(defines_begin EQUAL -1 OR first_field EQUAL -1 OR structs_end EQUAL -1)
    message(FATAL_ERROR "SITL packet structs not found in ${bf_target_h}")
endif()
string(SUBSTRING "${bf_content}" 0 ${first_field} before_first_field)
string(FIND "${before_first_field}" "typedef struct" structs_begin REVERSE)
string(SUBSTRING "${bf_content}" ${defines_begin} 100 bf_defines)
string(REGEX MATCHALL "#define SIMULATOR_MAX_[A-Z_]+ +[0-9]+" bf_defines "${bf_defines}")
string(REPLACE ";" "\n" bf_defines "${bf_defines}")
math(EXPR structs_length "${structs_end} - ${structs_begin} + 19")
string(SUBSTRING "${bf_content}" ${structs_begin} ${structs_length} bf_structs)

set(generated_dir ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(CONFIGURE OUTPUT ${generated_dir}/upstream_sitl_packets.hpp CONTENT
"#pragma once
#include <cstdint>
${bf_defines}
namespace upstream {
${bf_structs}
}  // namespace upstream
" @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${bf_target_h})

fpvsim_add_test(simcore_tests
    protocol_layout_test.cpp)
target_include_directories(simcore_tests PRIVATE
    ${PROJECT_SOURCE_DIR}/simcore/src
    ${generated_dir})
