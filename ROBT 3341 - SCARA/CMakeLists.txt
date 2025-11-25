cmake_minimum_required(VERSION 3.16)
project(test_code C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_BINARY_DIR}>)

link_libraries(gclib gclibo)
if (WIN32)
    include_directories($ENV{GCLIB_ROOT}/include)
    link_directories($ENV{GCLIB_ROOT}/lib/dynamic/x64 $ENV{GCLIB_ROOT}/lib/dynamic/x86)
endif ()

add_executable(scara_main scara_main.c)
