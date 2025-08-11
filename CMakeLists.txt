cmake_minimum_required(VERSION 3.10)
project(InertialLocalization)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

# Fetch Eigen library with exact version 3.4.0
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)

FetchContent_Declare(
    jsoncpp
    GIT_REPOSITORY https://github.com/open-source-parsers/jsoncpp.git
    GIT_TAG 1.9.5
)

# Add Google Test
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)

# Make all dependencies available
FetchContent_MakeAvailable(eigen jsoncpp googletest)

# Add Eigen include path
include_directories(${eigen_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${jsoncpp_SOURCE_DIR}/include)


# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

file(GLOB_RECURSE SOURCES "src/**.cpp" "src/**.h")

add_library(inertial_localization_api SHARED ${SOURCES})

target_link_libraries(inertial_localization_api PUBLIC jsoncpp_lib)

add_subdirectory(test_package)
