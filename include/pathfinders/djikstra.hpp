#ifndef COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_DIJKSTRA_HPP
#define COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_DIJKSTRA_HPP

#include "directed_graph.hpp"
//#include "Pathfinders.SharedUtils.hpp"
#include <algorithm>
#include <cstdlib>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <boost/container/flat_map.hpp>

namespace pathfinders {

    using EdgeSet = directed_weighted_edge_set<frozen_vertex_set<int>, double>;

    namespace detail {
        template <typename EdgeSet>
        using ancestor_map = boost::container::flat_map<typename EdgeSet::vertex_iterator_type, typename EdgeSet::edge_type>;

        path<typename EdgeSet::edge_type> rebuild_path(const ancestor_map<EdgeSet> &ancestor_map,
                                                       typename EdgeSet::vertex_iterator_type source,
                                                       typename EdgeSet::vertex_iterator_type destination) {
            path<typename EdgeSet::edge_type> result;
            auto current_vertex = destination;
            while (current_vertex != source) {
                const auto &edge = ancestor_map.find(current_vertex)->second;
                current_vertex = edge.get_from();
                result.push_back(edge);
            }

            std::reverse(result.begin(), result.end());
            return result;
        }
    }

//    template <typename EdgeSet>
    std::optional<path<typename EdgeSet::edge_type>> find_djikstra_shortest_path(const EdgeSet& edge_set,
                                                                                typename EdgeSet::vertex_iterator_type source,
                                                                                typename EdgeSet::vertex_iterator_type destination) {
        using vertex_iterator_type = typename EdgeSet::vertex_iterator_type;
        using weight_type = typename EdgeSet::weight_type;

        // distance from source to the vertex in key
        boost::container::flat_map<vertex_iterator_type, weight_type> distances;

        // which edge leads to the vertex in key
        detail::ancestor_map<EdgeSet> parent_map;

        struct distance_to_type {
            vertex_iterator_type vertex;
            weight_type distance;

            bool operator<(const distance_to_type& rhs) const {
                return distance < rhs.distance;
            }
        };

        boost::container::flat_set<vertex_iterator_type> visited_vertices;
        std::priority_queue<distance_to_type> vertices_to_visit;
        vertices_to_visit.push({source, weight_type(0)});
        while (!vertices_to_visit.empty()) {
            auto least_distance_vertex = vertices_to_visit.top();
            vertices_to_visit.pop();
            if (least_distance_vertex.vertex == destination) {
                return detail::rebuild_path(parent_map, source, destination);
            }

            if (visited_vertices.contains(least_distance_vertex.vertex)) {
                continue;
            }
            visited_vertices.insert(least_distance_vertex.vertex);
            auto outgoing_edges = edge_set.get_outgoing_edge_list(least_distance_vertex.vertex);
            // distance could have been updated, but the priority queue is not updated,
            // thus it is better to retrieve it from distances map
            auto distance_to_vertex = distances[least_distance_vertex.vertex];
            for (const auto& edge: outgoing_edges) {
                auto neighbor = edge.get_to();
                auto weight = edge.get_weight();
                if (visited_vertices.contains(edge.get_to())) {
                    continue;
                }

                if (auto distance_pos = distances.find(edge.get_to());
                    distance_pos != distances.end() && distance_to_vertex + weight >= distance_pos->second) {
                    continue;
                }

                distances[neighbor] = distance_to_vertex + weight;
                vertices_to_visit.push({
                    .vertex = neighbor,
                    .distance = distance_to_vertex + weight
                });
//                parent_map[neighbor] = edge;
                auto parent_location = parent_map.find(neighbor);
                if (parent_location == parent_map.end()) {
                    parent_map.emplace(neighbor, edge);
                } else {
                    parent_location->second = edge;
                }
            }
        }

        return {};
    }
}

#endif // COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_DIJKSTRA_HPP