add_library(fpvsim_options INTERFACE)
add_library(fpvsim::options ALIAS fpvsim_options)

target_compile_options(fpvsim_options INTERFACE
    -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wnon-virtual-dtor)

if(FPVSIM_WERROR)
    target_compile_options(fpvsim_options INTERFACE -Werror)
endif()

if(FPVSIM_SANITIZE)
    target_compile_options(fpvsim_options INTERFACE
        -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(fpvsim_options INTERFACE -fsanitize=address,undefined)
endif()

# Link after fpvsim::options so these follow -Wpedantic; both warnings come from doctest
add_library(fpvsim_test_options INTERFACE)
add_library(fpvsim::test_options ALIAS fpvsim_test_options)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(fpvsim_test_options INTERFACE -Wno-c2y-extensions "-Wno-#warnings")
endif()

# fpvsim_add_test(<name> <source>... LIBS <library>...)
function(fpvsim_add_test name)
    cmake_parse_arguments(ARG "" "" "LIBS" ${ARGN})
    add_executable(${name} ${ARG_UNPARSED_ARGUMENTS})
    target_link_libraries(${name} PRIVATE
        fpvsim::options fpvsim::test_options doctest::doctest ${ARG_LIBS})
    add_test(NAME ${name} COMMAND ${name})
endfunction()
