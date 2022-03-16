cmake_minimum_required(VERSION 3.20)
project(pathfinder)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

add_executable(demo main.cpp)
target_compile_features(demo PRIVATE cxx_std_20)

find_package(fmt REQUIRED)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)
find_package(CLI11 REQUIRED)

target_link_libraries(demo PRIVATE fmt::fmt)
target_link_libraries(demo PRIVATE Boost::headers)
target_link_libraries(demo PRIVATE Boost::json)
target_link_libraries(demo PRIVATE spdlog::spdlog)
target_link_libraries(demo PRIVATE CLI11::CLI11)