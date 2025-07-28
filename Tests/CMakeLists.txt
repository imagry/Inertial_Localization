set(TEST_SOURCES  test_setup.cpp)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

add_library(Tests ${TEST_SOURCES})

target_link_libraries(Tests Utils control_api_wrapper nlohmann_json::nlohmann_json)