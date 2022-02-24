#ifndef COM_GITHUB_CODERODDE_PATHFINDERS_UTIL_HPP
#define COM_GITHUB_CODERODDE_PATHFINDERS_UTIL_HPP

#include "DirectedGraph.hpp"
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

using namespace com::github::coderodde::directed_graph;

namespace com::github::coderodde::pathfinders::util {

    template<typename Node = int, typename Weight = double>
    class HeuristicFunction {
    public:
        virtual Weight estimate(Node const& tail, Node const& head) = 0;

        virtual ~HeuristicFunction() {

        }
    };

    template<typename Node = int, typename Weight = double>
    class Path {
    private:
        std::vector<Node> nodes_;
        DirectedGraphWeightFunction<Node, Weight> weight_function_;

    public:
        Path(std::vector<Node> const& nodes,
            DirectedGraphWeightFunction<Node, Weight> const& weight_function)
            :
            weight_function_{ weight_function }
        {
            for (Node e : nodes) {
                nodes_.push_back(e);
            }
        }

        Node operator[](std::size_t index) {
            return nodes_[index];
        }

        std::size_t length() {
            return nodes_.size();
        }

        Weight distance() {
            Weight total_distance = {};

            for (std::size_t i = 0; i < nodes_.size() - 1; ++i) {
                total_distance += 
                    weight_function_.getWeight(
                        nodes_[i], 
                        nodes_[i + 1]);
            }

            return total_distance;
        }
    };

    class PathDoesNotExistException : public std::logic_error {
    public:
        PathDoesNotExistException(std::string const& err_msg)
            :
            std::logic_error{ err_msg }
        {}
    };

    class NodeNotPresentInGraphException : public std::logic_error {
    public:
        NodeNotPresentInGraphException(std::string const& err_msg)
            :
            std::logic_error{ err_msg }
        {}
    };

    template<typename Node = int, typename Weight = double>
    struct HeapNode {
    private:
        Weight distance_;
        Node element_;

    public:
        HeapNode(Node const& element, Weight const& distance)
            :
            distance_{ distance },
            element_{ element }
        {

        }

        bool operator<(HeapNode<Node, Weight> other) const noexcept {
            return distance_ < other.distance_;
        }

        [[nodiscard]] Node getElement() const noexcept {
            return element_;
        }

        [[nodiscard]] Weight getDistance() const noexcept {
            return distance_;
        }
    };

    template<typename Node = int, typename Weight = double>
    class HeapNodeComparator {
    public:

        bool operator()(HeapNode<Node, Weight>* first,
            HeapNode<Node, Weight>* second) {
            return first->getDistance() > second->getDistance();
        }
    };

    template<typename Node = int>
    std::string buildSourceNodeNotInGraphErrorMessage(Node source_node) {
        std::stringstream ss;
        ss << "There is no source node " << source_node << " in the graph.";
        return ss.str();
    }

    template<typename Node = int>
    std::string buildTargetNodeNotInGraphErrorMessage(Node target_node) {
        std::stringstream ss;
        ss << "There is no target node " << target_node << " in the graph.";
        return ss.str();
    }

    template<typename Node = int>
    void checkTerminalNodes(DirectedGraph<Node> graph,
        Node source_node,
        Node target_node) {

        if (!graph.hasNode(source_node)) {
            throw NodeNotPresentInGraphException{
                buildSourceNodeNotInGraphErrorMessage(source_node)
            };
        }

        if (!graph.hasNode(target_node)) {
            throw NodeNotPresentInGraphException{
                buildTargetNodeNotInGraphErrorMessage(target_node)
            };
        }
    }

    template<typename Node = int>
    std::string buildPathNotExistsErrorMessage(Node source_node, Node target_node) {
        std::stringstream ss;
        ss << "There is no path from "
           << source_node 
           << " to " 
           << target_node 
           << ".";

        return ss.str();
    }

    template<typename Node = int, typename Weight = double>
    Path<Node, Weight>
        tracebackPath(Node& target_node,
            std::unordered_map<Node, Node*>& parent_map,
            DirectedGraphWeightFunction<Node, Weight>& weight_function) {

        std::vector<Node> path_nodes;
        Node previous_node = target_node;
        path_nodes.push_back(target_node);

        while (true) {
            Node* next_node = parent_map[previous_node];

            if (next_node == nullptr) {
                std::reverse(path_nodes.begin(), path_nodes.end());
                return Path<Node, Weight>{path_nodes, weight_function};
            }

            path_nodes.push_back(*next_node);
            previous_node = *next_node;
        }
    }

    template<typename Node = int, typename Weight = double>
    Path<Node, Weight>
        tracebackPath(
            const Node& touch_node,
            std::unordered_map<Node, Node*>& forward_parent_map,
            std::unordered_map<Node, Node*>& backward_parent_map,
            DirectedGraphWeightFunction<Node, Weight>& weight_function) {

        std::vector<Node> path_nodes;
        Node previous_node = touch_node;
        path_nodes.push_back(touch_node);

        while (true) {
            Node* next_node = forward_parent_map[previous_node];

            if (next_node == nullptr) {
                std::reverse(path_nodes.begin(), path_nodes.end());
                break;
            }

            path_nodes.push_back(*next_node);
            previous_node = *next_node;
        }

        Node* next_node = backward_parent_map[touch_node];

        while (next_node != nullptr) {
            path_nodes.push_back(*next_node);
            next_node = backward_parent_map[*next_node];
        }

        return Path<Node, Weight>{path_nodes, weight_function};
    }

    template<typename Node = int, typename Weight = double>
    void cleanPriorityQueue(
        std::priority_queue<HeapNode<Node, Weight>*,
        std::vector<HeapNode<Node, Weight>*>,
        HeapNodeComparator<Node, Weight>>&queue) {
        while (!queue.empty()) {
            HeapNode<Node, Weight>* heap_node = queue.top();
            queue.pop();
            delete heap_node;
        }
    }

    template<typename Node = int>
    void cleanParentMap(std::unordered_map<Node, Node*> parent_map) {
        for (const auto p : parent_map) {
            // One 'p.second' will be 'nullptr', but we can "delete" it too:
            delete p.second;
        }

        parent_map.clear();
    }

}; // End of namespace 'com::github::coderodde::pathfinders::util'.

#endif // COM_GITHUB_CODERODDE_PATHFINDERS_UTIL_HPP