cmake_minimum_required(VERSION 3.20)

project(aidriver
    LANGUAGES CXX)

add_library(localization_api_wrapper 
    ${CMAKE_CURRENT_SOURCE_DIR}/localization_api_wrapper.cpp
)

target_include_directories(localization_api_wrapper PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(localization_api_wrapper PUBLIC 
    LocalizationAPI
)
