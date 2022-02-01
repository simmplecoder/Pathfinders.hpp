#ifndef COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_NBA_STAR_HPP
#define COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_NBA_STAR_HPP

#include "DirectedGraph.hpp"
#include "Pathfinders.SharedUtils.hpp"
#include <algorithm>
#include <cstdlib>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace com::github::coderodde::pathfinders {

    using namespace com::github::coderodde::directed_graph;
    using namespace com::github::coderodde::pathfinders::util;

    template<typename Node = int, typename Weight = double> 
    void stabilizeForward(
        DirectedGraph<Node>& graph,
        DirectedGraphWeightFunction<Node, Weight>& weight_function,
        HeuristicFunction<Node, Weight>& heuristic_function,
        std::priority_queue<
            HeapNode<Node, Weight>*,
            std::vector<HeapNode<Node, Weight>*>,
            HeapNodeComparator<Node, Weight>>& OPEN_FORWARD,
        std::unordered_set<Node>& CLOSED,
        std::unordered_map<Node, Weight>& distance_map_forward,
        std::unordered_map<Node, Weight>& distance_map_backward,
        std::unordered_map<Node, Node*>& parent_map_forward,
        Node const& current_node,
        Node const& target_node,
        Weight& best_cost,
        const Node** touch_node_ptr) {

        std::unordered_set<Node>* children =
            graph.getChildNodesOf(current_node);

        for (Node const& child_node : *children) {
            if (CLOSED.contains(child_node)) {
                continue;
            }

            Weight tentative_distance =
                distance_map_forward[current_node] +
                weight_function.getWeight(current_node, child_node);

            if (!distance_map_forward.contains(child_node)
                || distance_map_forward[child_node] > tentative_distance) {
                OPEN_FORWARD.push(
                    new HeapNode<Node, Weight>(
                        child_node,
                        tentative_distance + 
                        heuristic_function.estimate(child_node, target_node)));

                distance_map_forward[child_node] = tentative_distance;
                Node* node_ptr = new Node{ current_node };
                parent_map_forward[child_node] = node_ptr;

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
        std::priority_queue<
            HeapNode<Node, Weight>*,
            std::vector<HeapNode<Node, Weight>*>,
            HeapNodeComparator<Node, Weight>>& OPEN_BACKWARD,
        std::unordered_set<Node>& CLOSED,
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
            if (CLOSED.contains(parent_node)) {
                continue;
            }

            Weight tentative_distance =
                distance_map_backward[current_node] +
                weight_function.getWeight(parent_node, current_node);

            if (!distance_map_backward.contains(parent_node)
                || distance_map_backward[parent_node] > tentative_distance) {
                OPEN_BACKWARD.push(
                    new HeapNode<Node, Weight>(
                        parent_node,
                        tentative_distance +
                        heuristic_function.estimate(parent_node, source_node)));

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

        std::priority_queue<
            HeapNode<Node, Weight>*,
            std::vector<HeapNode<Node, Weight>*>,
            HeapNodeComparator<Node, Weight>> OPEN_FORWARD;

        std::priority_queue<
            HeapNode<Node, Weight>*,
            std::vector<HeapNode<Node, Weight>*>,
            HeapNodeComparator<Node, Weight>> OPEN_BACKWARD;

        std::unordered_set<Node> CLOSED;

        std::unordered_map<Node, Weight> distance_map_forward;
        std::unordered_map<Node, Weight> distance_map_backward;

        std::unordered_map<Node, Node*> parent_map_forward;
        std::unordered_map<Node, Node*> parent_map_backward;

        OPEN_FORWARD .push(new HeapNode<Node, Weight>(source_node, Weight{}));
        OPEN_BACKWARD.push(new HeapNode<Node, Weight>(target_node, Weight{}));

        distance_map_forward[source_node] = Weight{};
        distance_map_backward[target_node] = Weight{};

        parent_map_forward[source_node] = nullptr;
        parent_map_backward[target_node] = nullptr;

        const Node* touch_node = nullptr;
        Weight best_cost = std::numeric_limits<Weight>::max();

        Weight total_distance =
            heuristic_function
            ->estimate(
                source_node,
                target_node);


        Weight f_cost_forward = total_distance;
        Weight f_cost_backward = total_distance;

        while (!OPEN_FORWARD.empty() && !OPEN_BACKWARD.empty()) {
            if (OPEN_FORWARD.size() < OPEN_BACKWARD.size()) {
                HeapNode<Node, Weight>* top_heap_node = OPEN_FORWARD.top();
                OPEN_FORWARD.pop();
                Node current_node = top_heap_node->getElement();
                delete top_heap_node;

                if (CLOSED.contains(current_node)) {
                    continue;
                }

                CLOSED.insert(current_node);

                if (distance_map_forward[current_node] +
                    heuristic_function->estimate(current_node, target_node)
                    >= best_cost
                    ||
                    distance_map_forward[current_node] + f_cost_backward
                    - heuristic_function->estimate(current_node, source_node)
                    >= best_cost) {
                    // Reject the 'current_node'!
                } else {
                    // Stabilize the 'current_node':
                    stabilizeForward<Node, Weight>(
                                     graph,
                                     weight_function,
                                     *heuristic_function,
                                     OPEN_FORWARD,
                                     CLOSED,
                                     distance_map_forward,
                                     distance_map_backward,
                                     parent_map_forward,
                                     current_node,
                                     target_node,
                                     best_cost,
                                     &touch_node);
                }

                if (!OPEN_FORWARD.empty()) {
                    f_cost_forward = OPEN_FORWARD.top()->getDistance();
                }
            } else {
                HeapNode<Node, Weight>* top_heap_node = OPEN_BACKWARD.top();
                OPEN_BACKWARD.pop();
                Node current_node = top_heap_node->getElement();
                delete top_heap_node;

                if (CLOSED.contains(current_node)) {
                    continue;
                }

                CLOSED.insert(current_node);

                if (distance_map_backward[current_node] +
                    heuristic_function->estimate(current_node, source_node)
                    >= best_cost
                    ||
                    distance_map_backward[current_node] + f_cost_forward
                    - heuristic_function->estimate(current_node, target_node)
                    >= best_cost) {
                    // Reject the 'current_node'!
                }
                else {
                    // Stabilize the 'current_node':
                    stabilizeBackward<Node, Weight>(
                                      graph,
                                      weight_function,
                                      *heuristic_function,
                                      OPEN_BACKWARD,
                                      CLOSED,
                                      distance_map_forward,
                                      distance_map_backward,
                                      parent_map_backward,
                                      current_node,
                                      source_node,
                                      best_cost,
                                      &touch_node);
                }

                if (!OPEN_BACKWARD.empty()) {
                    f_cost_backward = OPEN_BACKWARD.top()->getDistance();
                }
            }
        }

        cleanPriorityQueue(OPEN_FORWARD);
        cleanPriorityQueue(OPEN_BACKWARD);

        if (touch_node == nullptr) {
            cleanParentMap(parent_map_forward);
            cleanParentMap(parent_map_backward);

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