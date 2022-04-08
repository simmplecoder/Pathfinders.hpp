#include <catch2/catch.hpp>
#include <pathfinders/directed_graph.hpp>
#include <pathfinders/djikstra.hpp>

TEST_CASE("basic one edge Djikstra test") {
    pathfinders::vertex_set<int> vertices;
    vertices.insert(0);
    vertices.insert(1);
    auto frozen_vertices = pathfinders::frozen_vertex_set<int>(std::move(vertices));
    pathfinders::directed_weighted_edge_set<decltype(frozen_vertices), double> edges(frozen_vertices);
    edges.add_edge(frozen_vertices.iterator_for(0).value(), frozen_vertices.iterator_for(1).value(), 1.0);
    const auto path = pathfinders::find_djikstra_shortest_path(edges,
                                                               frozen_vertices.iterator_for(0).value(),
                                                               frozen_vertices.iterator_for(1).value());
    REQUIRE(path.has_value());
    REQUIRE(path->size() == 1);
    auto& edge = path.value()[0];
    REQUIRE(edge.get_to() == frozen_vertices.iterator_for(1));
    REQUIRE(edge.get_from() == frozen_vertices.iterator_for(0));
    REQUIRE(edge.get_weight() == 1.0);
}


