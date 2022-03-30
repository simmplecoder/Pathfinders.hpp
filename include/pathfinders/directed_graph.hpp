#ifndef COM_GITHUB_CODERODDE_DIRECTED_GRAPH_HPP
#define COM_GITHUB_CODERODDE_DIRECTED_GRAPH_HPP

#include <cstddef> // for std::size_t
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <boost/json.hpp>
#include <boost/container/flat_set.hpp>
#include <optional>
#include <span>

namespace pathfinders {
    template <typename ValueType, typename Compare = std::less<ValueType>, typename Allocator = boost::container::new_allocator<ValueType>>
    using vertex_set = boost::container::flat_set<ValueType, Compare, Allocator>;

    template <typename ValueType, typename Compare = std::less<ValueType>, typename Allocator = boost::container::new_allocator<ValueType>>
    class frozen_vertex_set {
        using container = vertex_set<ValueType, Compare, Allocator>;
        container frozen_vertices;
    public:
        using value_type = typename container::value_type;
        using key_type = typename container::key_type;
        using key_compare = typename container::key_compare;
        using iterator = typename container::const_iterator;
        using const_iterator = iterator;

        frozen_vertex_set(vertex_set<ValueType, Compare, Allocator>&& vertices_to_freeze) noexcept(std::is_nothrow_move_constructible_v<container>) :
                frozen_vertices(std::move(vertices_to_freeze))
        {}

        bool contains(const ValueType& vertex) const noexcept {
            return frozen_vertices.count(vertex) == 1;
        }

        std::optional<iterator> iterator_for(const ValueType& vertex) const noexcept {
            auto pos = frozen_vertices.find(vertex);
            if (pos == frozen_vertices.end()) {
                return {};
            } else {
                return pos;
            }
        }
    };

    template <typename FrozenGraph, typename WeightType>
    class directed_weighted_edge_set {
        const FrozenGraph* frozen_graph;

    public:
        using vertex_iterator_type = typename FrozenGraph::iterator;
        using weight_type = WeightType;

        class edge_type {
            vertex_iterator_type from;
            vertex_iterator_type to;
            weight_type weight;
        public:
            static constexpr bool is_weighted = true;
            static constexpr bool is_bidirectional = true;

            vertex_iterator_type get_from() const noexcept {
                return from;
            }

            vertex_iterator_type get_to() const noexcept {
                return to;
            }

            weight_type get_weight() const noexcept {
                return weight;
            }

            friend bool operator<(const edge_type& lhs, const edge_type& rhs) {
                if (lhs.from == rhs.from) {
                    return lhs.to < rhs.to;
                } else {
                    return lhs.from < rhs.from;
                }
            }

        private:
            edge_type(vertex_iterator_type from, vertex_iterator_type to, weight_type weight):
                    from(from),
                    to(to),
                    weight(weight)
            {}

            static edge_type create_opaque_edge(vertex_iterator_type source_vertex) {
                return edge_type(source_vertex, {}, 0);
            }

            friend directed_weighted_edge_set;
        };

        directed_weighted_edge_set(const FrozenGraph& target):
                frozen_graph(std::addressof(target))
        {}

        bool add_edge(vertex_iterator_type from,
                      vertex_iterator_type to,
                      weight_type weight) {
            return edges.insert(edge_type(from, to, weight)).second;
        }

        std::span<const edge_type, std::dynamic_extent> get_outgoing_edge_list(vertex_iterator_type vertex) const noexcept {
            auto opaque_edge = edge_type::create_opaque_edge(vertex);
            auto edge_list_start = std::lower_bound(
                    edges.begin(),
                    edges.end(),
                    opaque_edge,
                    [](const edge_type& lhs, const edge_type& rhs) {
                        return lhs.from < rhs.from;
                    });

            if (edge_list_start->from != vertex) {
                return {};
            }

            auto edge_list_end = std::upper_bound(
                    edges.begin(),
                    edges.end(),
                    opaque_edge,
                    [](const edge_type& lhs, const edge_type& rhs) {
                        return lhs.from < rhs.from;
                    });

            return std::span<const edge_type, std::dynamic_extent>(edge_list_start, edge_list_end);
        }

    private:
        boost::container::flat_set<edge_type> edges;
    };

    template <typename EdgeSet>
    using edge_type = typename EdgeSet::edge_type;

    template <typename EdgeType>
    using path = std::vector<EdgeType>;

    template <typename ValueType, typename WeightType>
    using directed_weighted_path = path<typename directed_weighted_edge_set<frozen_vertex_set<ValueType>,  WeightType>::edge_type>;
}

//namespace com::github::coderodde::directed_graph {
//
//
//    template<typename Node = int>
//    class DirectedGraph {
//    private:
//        std::unordered_set<Node> nodes_;
//
//    public:
//        using Iterator = typename std::unordered_set<Node>::const_iterator;
//
//        struct InsertResult {
//            bool successful;
//            Iterator iterator; // always valid, either points to new element or existing one
//            explicit operator bool() {
//                return successful;
//            }
//        };
//
//        Iterator addNode(Node const& node) {
//            return nodes_.insert(node).second;
//        }
//
//        bool hasNode(Node const& node) {
//            return nodes_.contains(node);
//        }
//
//        bool removeNode(Node const& node) {
//            nodes_.erase(node);
//        }
//
//        bool addArc(Node const& tail, Node const& head) {
//            bool state_changed = false;
//
//            if (!hasNode(tail)) {
//                addNode(tail);
//                state_changed = true;
//            }
//
//            if (!hasNode(head)) {
//                addNode(head);
//                state_changed = true;
//            }
//
//            if (!child_map_[tail].contains(head)) {
//                child_map_[tail].insert(head);
//                state_changed = true;
//            }
//
//            if (!parent_map_[head].contains(tail)) {
//                parent_map_[head].insert(tail);
//                state_changed = true;
//            }
//
//            if (state_changed) {
//                number_of_arcs_++;
//            }
//
//            return state_changed;
//        }
//
//        bool hasArc(Node const& tail, Node const& head) {
//            if (!child_map_.contains(tail)) {
//                return false;
//            }
//
//            return child_map_[tail].contains(head);
//        }
//
//        bool removeArc(Node const& tail, Node const& head) {
//            if (!child_map_.contains(tail)) {
//                return false;
//            }
//
//            if (!child_map_[tail].contains(head)) {
//                return false;
//            }
//
//            child_map_[tail].erase(head);
//            parent_map_[head].erase(tail);
//            number_of_arcs_--;
//            return true;
//        }
//
//        std::unordered_set<Node>* getParentNodesOf(Node const& node) {
//            return &parent_map_[node];
//        }
//
//        std::unordered_set<Node>* getChildNodesOf(Node const& node) {
//            return &child_map_[node];
//        }
//
//        std::unordered_set<Node> const& getNodes() const noexcept {
//            return nodes_;
//        }
//
//        [[nodiscard]] std::size_t getNumberOfNodes() const noexcept {
//            return nodes_.size();
//        }
//
//        [[nodiscard]] std::size_t getNumberOfArcs() const noexcept {
//            return number_of_arcs_;
//        }
//
//        [[nodiscard]] boost::json::object toJSON() const {
//            boost::json::object result;
//
//            boost::json::array nodes_array;
//            for (const auto& node: nodes_) {
//                nodes_array.push_back(node);
//            }
//
//            result["nodes"] = std::move(nodes_array);
//
//            boost::json::array arcs_array;
//            for (const auto& [node, parents]: parent_map_) {
//                for (const auto& parent: parents) {
//                    boost::json::object arc_desc;
//                    arc_desc["to"] = node;
//                    arc_desc["from"] = parent;
//                    arcs_array.push_back(arc_desc);
//                }
//            }
//
//            result["arcs"] = arcs_array;
//            return result;
//        }
//    };
//
//    template<typename Node = int>
//    std::string buildNonExistingArcErrorMessage(
//        Node const& tail,
//        Node const& head) {
//
//        std::stringstream ss;
//        ss << "The arc (" << tail << ", " << head << ") does not exist.";
//        return ss.str();
//    }
//
//    class NonExistingArcException : public std::logic_error {
//    public:
//        NonExistingArcException(std::string const& err_msg)
//            :
//            std::logic_error{ err_msg }
//        {}
//    };
//
//    template<typename Node = int, typename Weight = double>
//    class DirectedGraphWeightFunction {
//    private:
//        std::unordered_map<Node, std::unordered_map<Node, Weight>> weight_map_;
//
//    public:
//        void addWeight(Node const& parent, Node const& child, Weight weight) {
//            weight_map_[parent][child] = weight;
//        }
//
//        void removeWeight(Node const& parent, Node const& child) {
//            if (!weight_map_.contains(parent)
//                || !weight_map_[parent].contains(child)) {
//                return;
//            }
//
//            weight_map_[parent].erase(child);
//        }
//
//        Weight getWeight(Node const& parent, Node const& child) {
//            if (!weight_map_.contains(parent)
//                || !weight_map_[parent].contains(child)) {
//                throw NonExistingArcException{
//                    buildNonExistingArcErrorMessage(parent, child)
//                };
//            }
//
//            return weight_map_[parent][child];
//        }
//
//        boost::json::array toJSON() const {
//            boost::json::array weights;
//            for (const auto& [parent, arcs]: weight_map_) {
//                for (const auto& [child, arc_weight]: arcs) {
//                    boost::json::object arc;
//                    arc["from"] = parent;
//                    arc["to"] = child;
//                    arc["weight"] = arc_weight;
//
//                    weights.push_back(arc);
//                }
//            }
//
//            return weights;
//        }
//    };
//} // End of namespace com::github::coderodde::directed_graph.

#endif // COM_GITHUB_CODERODDE_DIRECTED_GRAPH_HPP