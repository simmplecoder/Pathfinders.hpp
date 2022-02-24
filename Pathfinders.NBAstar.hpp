#ifndef COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_NBA_STAR_HPP
#define COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_NBA_STAR_HPP

#include "DirectedGraph.hpp"
#include "Pathfinders.SharedUtils.hpp"
#include <algorithm>
#include <cstdlib>
#include <queue>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace com::github::coderodde::pathfinders {

    template<typename Node = int, typename Weight = double>
    struct Info {
        bool closed;
        std::optional<Weight> distance_forward;
        std::optional<Weight> distance_backward;
        std::optional<Node> parent_forward;
        std::optional<Node> parent_backward;

        Info() : closed{false},
            distance_forward { std::nullopt },
            distance_backward{ std::nullopt },
            parent_forward { std::nullopt },
            parent_backward{ std::nullopt }, {
        }
    };

    using namespace com::github::coderodde::directed_graph;
    using namespace com::github::coderodde::pathfinders::util;

    template<typename Node = int, typename Weight = double> 
    void stabilizeForward(
        DirectedGraph<Node>& graph,
        DirectedGraphWeightFunction<Node, Weight>& weight_function,
        HeuristicFunction<Node, Weight>& heuristic_function,
        std::priority_queue<HeapNode<Node, Weight>>& open_forward,
        std::unordered_map<Node, Info<Node, Weight>>& info,
        Node const& current_node,
        Node const& target_node,
        Weight& best_cost,
        const Node** touch_node_ptr) {

        std::unordered_set<Node>* children =
            graph.getChildNodesOf(current_node);

        for (Node const& child_node : *children) {
            if (info[child_node].closed) {
                continue;
            }

            Weight tentative_distance =
                info[current_node].distance_forward +
                weight_function.getWeight(current_node, child_node);

            if (!info.contains(child_node) 
                || info[child_node].distance_forward > tentative_distance) {

                HeapNode<Node, Weight> 
                    node{child_node, 
                         tentative_distance + 
                         heuristic_function.estimate(child_node, target_node)};
                
                open_forward.emplace(node);

                info[child_node].distance_forward = tentative_distance;
                info[child_node].parent_forward = current_node;

                if (info)
                if (distance_map_backward.contains(child_node)) {
                    Weight path_length = tentative_distance +
                        distance_map_backward[child_node];

                    if (best_cost > path_length) {
                        best_cost = path_length;
                        *touch_node_ptr = &child_node;
                    }
                }
            }
        }
    }

    template<typename Node = int, typename Weight = double>
    void stabilizeBackward(
        DirectedGraph<Node>& graph,
        DirectedGraphWeightFunction<Node, Weight>& weight_function,
        HeuristicFunction<Node, Weight>& heuristic_function,
        std::priority_queue<HeapNode<Node, Weight>>& open_backward,
        std::unordered_set<Node>& closed,
        std::unordered_map<Node, Weight>& distance_map_forward,
        std::unordered_map<Node, Weight>& distance_map_backward,
        std::unordered_map<Node, Node*>& parent_map_backward,
        Node const& current_node,
        Node const& source_node,
        Weight& best_cost,
        const Node** touch_node_ptr) {

        std::unordered_set<Node>* parents =
            graph.getParentNodesOf(current_node);

        for (Node const& parent_node : *parents) {
            if (closed.contains(parent_node)) {
                continue;
            }

            Weight tentative_distance =
                distance_map_backward[current_node] +
                weight_function.getWeight(parent_node, current_node);

            if (!distance_map_backward.contains(parent_node)
                || distance_map_backward[parent_node] > tentative_distance) {
                HeapNode<Node, Weight>
                    node{ parent_node,
                         tentative_distance +
                         heuristic_function.estimate(parent_node, source_node) };

                open_backward.emplace(node);

                distance_map_backward[parent_node] = tentative_distance;
                Node* node_ptr = new Node{ current_node };
                parent_map_backward[parent_node] = node_ptr;

                if (distance_map_forward.contains(parent_node)) {
                    Weight path_length = tentative_distance +
                        distance_map_forward[parent_node];

                    if (best_cost > path_length) {
                        best_cost = path_length;
                        *touch_node_ptr = &parent_node;
                    }
                }
            }
        }
    }

    template<typename Node = int, typename Weight = double>
    Path<Node, Weight>
        runBidirectionalAstarAlgorithm(
            DirectedGraph<Node>& graph,
            DirectedGraphWeightFunction<Node, Weight>& weight_function,
            HeuristicFunction<Node, Weight>* heuristic_function,
            Node& source_node,
            Node& target_node) {

        checkTerminalNodes(graph, source_node, target_node);

        std::priority_queue<HeapNode<Node, Weight>> open_forward;
        std::priority_queue<HeapNode<Node, Weight>> open_backward;
        std::unordered_map<Node, Info<Node, Weight>> info;

        open_forward .emplace(source_node, Weight{});
        open_backward.emplace(target_node, Weight{});

        info[source_node].distance_forward  = Weight{};
        info[target_node].distance_backward = Weight{};

        info[source_node].parent_forward  = std::nullopt;
        info[target_node].parent_backward = std::nullopt;

        const Node* touch_node = nullptr;
        Weight best_cost = std::numeric_limits<Weight>::max();

        Weight total_distance =
            heuristic_function
            ->estimate(
                source_node,
                target_node);

        Weight f_cost_forward = total_distance;
        Weight f_cost_backward = total_distance;

        while (!open_forward.empty() && !open_backward.empty()) {
            if (open_forward.size() < open_backward.size()) {
                Node current_node = open_forward.top().getElement();
                open_forward.pop();

                if (info[current_node].closed) {
                    continue;
                }

                info[current_node].closed = true;

                if (info[current_node].distance_forward +
                        heuristic_function->
                            estimate(current_node, target_node) >= best_cost
                    ||
                    info[current_node].distance_forward +
                        f_cost_backward -
                            heuristic_function->
                                estimate(current_node, source_node)) {
                    // Reject the 'current_node'.
                } else {
                    // Stabilize the 'current_node':
                    stabilizeForward<Node, Weight>(
                                     graph,
                                     weight_function,
                                     *heuristic_function,
                                     open_forward,
                                     info,
                                     current_node,
                                     target_node,
                                     best_cost,
                                     &touch_node);
                }

                if (!open_forward.empty()) {
                    f_cost_forward = open_forward.top().getDistance();
                }
            } else {
                Node current_node = open_backward.top().getElement();
                open_backward.pop();

                if (info[current_node].closed) {
                    continue;
                }

                int[current_node].closed = true;

                if (info[current_node].distance_backward + 
                    heuristic_function  
                    ->estimate(current_node, source_node)
                    >= best_cost 
                    ||
                    info[current_node].distance_backward + f_cost_forward -
                    heuristic_function->estimate(current_node, target_node) {
                    // Reject the 'current_node'!
                } else {
                    // Stabilize the 'current_node':
                    stabilizeBackward<Node, Weight>(
                                      graph,
                                      weight_function,
                                      *heuristic_function,
                                      open_backward,
                                      info,
                                      current_node,
                                      source_node,
                                      best_cost,
                                      &touch_node);
                }

                if (!open_backward.empty()) {
                    f_cost_backward = open_backward.top().getDistance();
                }
            }
        }

        if (touch_node == nullptr) {
            throw PathDoesNotExistException{
                buildPathNotExistsErrorMessage(source_node, target_node)
            };
        }

        Path<Node, Weight> path =
            tracebackPath(
                *touch_node,
                parent_map_forward,
                parent_map_backward,
                weight_function);

        cleanParentMap(parent_map_forward);
        cleanParentMap(parent_map_backward);
        return path;
    }
} // End of namespace 'com::github::coderodde::pathfinders'.

#endif // COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_NBA_STAR_HPP