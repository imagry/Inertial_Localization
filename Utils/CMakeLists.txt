set(SOURCE_FILES
  Functions.cpp
  Classes.cpp
  Controllers.cpp
  control_debug_states.cpp
  DataHandling.cpp
  AHRS.cpp
  short_term_localization.cpp
  SpeedEstimators.cpp
)
include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

add_library(Utils ${SOURCE_FILES})
# IMPORTANT: WHEN ADDING SOURCE FILES, MUST ADD TO CONTROL_API'S CMAKE LIST TOO
# add_library(Utils Functions.cpp Classes.cpp Controllers.cpp DataHandling.cpp AHRS.cpp)
target_link_libraries(Utils PUBLIC nlohmann_json::nlohmann_json)
target_include_directories(Utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})