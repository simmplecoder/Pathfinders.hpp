//#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <pathfinders/directed_graph.hpp>

TEST_CASE("basic tests for vertex insertion") {
    pathfinders::vertex_set<int> vertices;
    for (int i = 0; i < 1'000; ++i) {
        vertices.insert(i);
    }

    auto frozen_vertices = pathfinders::frozen_vertex_set<int>(std::move(vertices));
    for (int i = 0; i < 1'000; ++i) {
        REQUIRE(frozen_vertices.contains(i));
    }

    for (int i = 1'000; i < 2'000; ++i) {
        REQUIRE_FALSE(frozen_vertices.contains(i));
    }
}

TEST_CASE("vertex comparison after insertion") {
    pathfinders::vertex_set<int> vertices;
    for (int i = 0; i < 1'000; ++i) {
        vertices.insert(i);
    }

    auto frozen_vertices = pathfinders::frozen_vertex_set<int>(std::move(vertices));
    auto iterator0_0 = frozen_vertices.iterator_for(0);
    auto iterator0_1 = frozen_vertices.iterator_for(0);
    auto iterator1_0 = frozen_vertices.iterator_for(1);
    REQUIRE(iterator0_0 == iterator0_1);
    REQUIRE_FALSE(iterator0_0 == iterator1_0);
    REQUIRE_FALSE(iterator0_1 == iterator1_0);
}

TEST_CASE("edge insertion test") {
    pathfinders::vertex_set<int> vertices;
    for (int i = 0; i < 1'000; ++i) {
        vertices.insert(i);
    }

    auto frozen_vertices = pathfinders::frozen_vertex_set<int>(std::move(vertices));
    pathfinders::directed_weighted_edge_set<decltype(frozen_vertices)::iterator, double> edges;
    edges.add_edge(frozen_vertices.iterator_for(0), frozen_vertices.iterator_for(1), 1.0);
    edges.add_edge(frozen_vertices.iterator_for(0), frozen_vertices.iterator_for(2), 1.0);
    auto neighbor_list = edges.get_outgoing_edge_list(frozen_vertices.iterator_for(0));
    REQUIRE(neighbor_list.size() == 2);
    REQUIRE(neighbor_list[0].get_weight() == 1.0);
    REQUIRE(neighbor_list[0].get_from() == frozen_vertices.iterator_for(0));
    REQUIRE(neighbor_list[0].get_to() == frozen_vertices.iterator_for(1));
    REQUIRE(neighbor_list[1].get_weight() == 1.0);
    REQUIRE(neighbor_list[1].get_from() == frozen_vertices.iterator_for(0));
    REQUIRE(neighbor_list[1].get_to() == frozen_vertices.iterator_for(2));
}
