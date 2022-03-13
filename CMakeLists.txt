cmake_minimum_required(VERSION 3.20)
project(pathfinder)

add_executable(demo main.cpp)
target_compile_features(demo PRIVATE cxx_std_20)

find_package(fmt REQUIRED
        CONFIG
        PATHS /home/shino/software/bin/fmt/)
target_link_libraries(demo PRIVATE fmt::fmt)