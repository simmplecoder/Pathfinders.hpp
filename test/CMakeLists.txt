find_package(Catch2 REQUIRED)
include(Catch)

set(test_names
        basic_graph_tests
        djikstra_tests)

foreach(test_name ${test_names})
    add_executable(${test_name} ${test_name}.cpp)
    target_link_libraries(${test_name} PUBLIC Catch2::Catch2WithMain pathfinders)
    catch_discover_tests(${test_name})
endforeach()

#add_executable(basic_graph_tests basic_graph_tests.cpp)
#target_link_libraries(basic_graph_tests PUBLIC Catch2::Catch2WithMain pathfinders)
#
#catch_discover_tests(basic_graph_tests)
