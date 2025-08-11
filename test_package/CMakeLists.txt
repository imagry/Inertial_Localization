# Include gtest
include(GoogleTest)

# Discover and add tests to the test runner
file(GLOB TEST_SOURCES "inertial_localization_api/*.cpp")
set(TEST_PACKAGE_DIR ${CMAKE_CURRENT_LIST_DIR})
message(STATUS "Test package directory: ${TEST_PACKAGE_DIR}")
set(TEST_RESOURCES ${TEST_PACKAGE_DIR}/resources)

add_executable(run_tests ${TEST_SOURCES})

target_link_libraries(run_tests
    gtest
    gtest_main
    gmock
    gmock_main
    inertial_localization_api
)

# Enable testing for CTest integration
enable_testing()
add_test(NAME AllTests COMMAND run_tests)

add_subdirectory(${TEST_RESOURCES}/extern/pybind11)
# pybind11 is added as a subdirectory, so no need to find_package it.
pybind11_add_module(localization_pybind_module
    inertial_localization_api/python/python_binding/localization_pybind_module.cpp
)

target_link_libraries(localization_pybind_module PRIVATE pybind11::headers inertial_localization_api)

target_include_directories(localization_pybind_module PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${Python_INCLUDE_DIRS}
    ${TEST_RESOURCES}/extern/pybind11/include
)