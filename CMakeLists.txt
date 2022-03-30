cmake_minimum_required(VERSION 3.20)
project(pathfinder)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

find_package(Boost REQUIRED)

add_library(pathfinders INTERFACE)
target_include_directories(pathfinders INTERFACE include)
target_compile_features(pathfinders INTERFACE cxx_std_20)
target_link_libraries(pathfinders INTERFACE Boost::headers Boost::json)


add_executable(demo main.cpp)
target_link_libraries(demo PRIVATE pathfinders)

find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(CLI11 REQUIRED)

target_link_libraries(demo PRIVATE fmt::fmt)
target_link_libraries(demo PRIVATE spdlog::spdlog)
target_link_libraries(demo PRIVATE CLI11::CLI11)

include(CTest)
if (BUILD_TESTING)
    add_subdirectory(test)
endif()
