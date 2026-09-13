# ============================================================================
# EAE_Firmware
#
# Cooling loop firmware demonstration: simulated CAN bus, PID control,
# state machine, and unit tests.
#
# External dependencies (GoogleTest, CLI11) are downloaded at configure time
# by FetchContent. They are NOT committed into this repository, which
# satisfies the "do not ship dependencies" requirement.
# ============================================================================

cmake_minimum_required(VERSION 3.20)

project(EAE_Firmware
        VERSION 1.0.0
        DESCRIPTION "Cooling loop firmware for the EAE vehicle"
        LANGUAGES CXX)

# Use C++17 and refuse to silently fall back to an older standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to a Debug build if the user did not pick one.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ----------------------------------------------------------------------------
# External dependencies
# ----------------------------------------------------------------------------
include(FetchContent)

# CLI11: command line argument parsing (header-only).
FetchContent_Declare(
    cli11
    URL https://github.com/CLIUtils/CLI11/archive/refs/tags/v2.4.2.tar.gz
)

# GoogleTest: unit testing framework.
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
)

FetchContent_MakeAvailable(cli11 googletest)

# ----------------------------------------------------------------------------
# Core library
#
# The control logic lives in a library so that both the application and the
# unit tests can link against exactly the same code.
# ----------------------------------------------------------------------------
add_library(eae_core
    src/pid.cpp
    src/state_machine.cpp
)

target_include_directories(eae_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ----------------------------------------------------------------------------
# Application
# ----------------------------------------------------------------------------
add_executable(eae_firmware src/main.cpp)
target_link_libraries(eae_firmware PRIVATE eae_core CLI11::CLI11)

# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
enable_testing()
include(GoogleTest)

add_executable(eae_tests
    tests/test_pid.cpp
    tests/test_state_machine.cpp
)
target_link_libraries(eae_tests PRIVATE eae_core GTest::gtest_main)

gtest_discover_tests(eae_tests)
