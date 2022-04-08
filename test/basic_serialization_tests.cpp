#include <catch2/catch.hpp>
#include <pathfinders/directed_graph.hpp>
#include <pathfinders/serialization.hpp>
#include <fstream>

std::string json_str = R"start(
{
  "vertices": [
    1,
    2,
    3
    ],
  "edges": [
    {
      "from": 1,
      "to": 2,
      "weight": 1.0
    },
    {
      "from": 2,
      "to": 3,
      "weight": 2.0
    },
    {
      "from": 3,
      "to": 1,
      "weight": 3.0
    }
    ]
}
)start";

TEST_CASE("basic serialization") {
    auto json = boost::json::parse(json_str).as_object();
    auto vertices = pathfinders::frozen_vertex_set<int>(pathfinders::deserialize_vertices<int>(json["vertices"].as_array()));
    for (int i = 1; i <= 3; ++i) {
        REQUIRE(vertices.contains(i));
    }
    auto iter_0 = vertices.iterator_for(1);
    auto iter_1 = vertices.iterator_for(2);
    auto iter_2 = vertices.iterator_for(3);

    auto edges = pathfinders::deserialize_edge_set(
            vertices,
            json["edges"].as_array());

    auto neighbors_0 = edges.get_outgoing_edge_list(iter_0);
    REQUIRE(neighbors_0.size() == 1);
    REQUIRE(neighbors_0[0].get_from() == iter_0);
    REQUIRE(neighbors_0[0].get_to() == iter_1);



    auto neighbors_1 = edges.get_outgoing_edge_list(iter_1);
    REQUIRE(neighbors_1.size() == 1);
    REQUIRE(neighbors_1[0].get_from() == iter_1);
    REQUIRE(neighbors_1[0].get_to() == iter_2);
}

TEST_CASE("basic deserialization test") {
    auto vertices = pathfinders::vertex_set<int>{1, 2, 3};
    auto frozen_vertices = pathfinders::frozen_vertex_set(std::move(vertices));
    pathfinders::directed_weighted_edge_set<decltype(frozen_vertices)::iterator, double> edges;
    edges.add_edge(frozen_vertices.iterator_for(1), frozen_vertices.iterator_for(2), 1.0);
    edges.add_edge(frozen_vertices.iterator_for(2), frozen_vertices.iterator_for(3), 2.0);
    edges.add_edge(frozen_vertices.iterator_for(3), frozen_vertices.iterator_for(1), 3.0);

    auto vertices_json = pathfinders::serialize_vertices(frozen_vertices);
    auto test_json = boost::json::parse(json_str).as_object();
    REQUIRE(vertices_json == test_json["vertices"]);

    auto edges_json = pathfinders::serialize_edge_set(frozen_vertices, edges);
    REQUIRE(edges_json == test_json["edges"]);
}
