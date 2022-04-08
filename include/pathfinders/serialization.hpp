#ifndef PATHFINDER_SERIALIZATION_HPP
#define PATHFINDER_SERIALIZATION_HPP

#include <pathfinders/directed_graph.hpp>
#include <boost/json.hpp>
#include <type_traits>

namespace pathfinders {
    template <typename ValueType,
              typename Compare = std::less<ValueType>,
              typename Allocator = boost::container::new_allocator<ValueType>
    >
    vertex_set<ValueType, Compare, Allocator> deserialize_vertices(const boost::json::array& vertex_array) {
        vertex_set<ValueType, Compare, Allocator> vertices;
        for (const auto& vertex: vertex_array) {
            vertices.insert(boost::json::value_to<ValueType>(vertex));
        }

        return vertices;
    }

    template <typename ValueType,
            typename Compare = std::less<ValueType>,
            typename Allocator = boost::container::new_allocator<ValueType>
    >
    boost::json::array serialize_vertices(const frozen_vertex_set<ValueType, Compare, Allocator>& vertices) {
        boost::json::array result;
        result.reserve(vertices.size());
        boost::json::value v;
        for (const auto& vertex: vertices) {
//            result.push_back(serialization_traits<ValueType>::to_json(vertex));
            boost::json::value_from(vertex, v);
            result.push_back(v);
        }

        return result;
    }

    template <typename ValueType,
              typename Compare = std::less<ValueType>,
              typename Allocator = boost::container::new_allocator<ValueType>,
              typename WeightType = double
    >
    directed_weighted_edge_set<typename frozen_vertex_set<ValueType, Compare, Allocator>::iterator, WeightType> deserialize_edge_set(
            const frozen_vertex_set<ValueType, Compare, Allocator>& vertex_set,
            boost::json::array& edge_array
            ) {
        directed_weighted_edge_set<typename ::pathfinders::frozen_vertex_set<ValueType, Compare, Allocator>::iterator, WeightType> result;
        boost::json::value v;
        for (auto& edge: edge_array) {
            auto& object = edge.as_object();
            auto from = boost::json::value_to<ValueType>(object["from"]);
            auto to = boost::json::value_to<ValueType>(object["to"]);
            auto weight = boost::json::value_to<WeightType>(object["weight"]);
            result.add_edge(vertex_set.iterator_for(from), vertex_set.iterator_for(to), weight);
        }

        return result;
    }


    template <typename ValueType,
            typename Compare = std::less<ValueType>,
            typename Allocator = boost::container::new_allocator<ValueType>,
            typename WeightType = double
    >
    boost::json::array serialize_edge_set(
            const frozen_vertex_set<ValueType, Compare, Allocator>& vertex_set,
            const directed_weighted_edge_set<typename frozen_vertex_set<ValueType, Compare, Allocator>::iterator, WeightType>& edge_set
            ) {
        boost::json::array result;
        boost::json::object edge_json;
        boost::json::value vertex_value;
        boost::json::value weight_value;
        for (auto vertex = vertex_set.begin(); vertex != vertex_set.end(); ++vertex) {
            auto neighbors = edge_set.get_outgoing_edge_list(vertex);
            for (const auto& neighbor: neighbors) {
                boost::json::value_from(*vertex, vertex_value);
                edge_json["from"] = vertex_value;
                boost::json::value_from(*neighbor.get_to(), vertex_value);
                edge_json["to"] = vertex_value;
                boost::json::value_from(neighbor.get_weight(), weight_value);
                edge_json["weight"] = weight_value;
                result.push_back(edge_json);
            }
        }

        return result;
    }
}

#endif //PATHFINDER_SERIALIZATION_HPP
