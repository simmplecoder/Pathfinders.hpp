#ifndef COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_ASTAR_HPP
#define COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_ASTAR_HPP

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
    Path<Node, Weight>
        runAstarAlgorithm(
            DirectedGraph<Node>& graph,
            DirectedGraphWeightFunction<Node, Weight>& weight_function,
            HeuristicFunction<Node, Weight>* heuristic_function,
            Node& source_node,
            Node& target_node) {

        checkTerminalNodes(graph, source_node, target_node);

        std::priority_queue<
            HeapNode<Node, Weight>*,
            std::vector<HeapNode<Node, Weight>*>,
            HeapNodeComparator<Node, Weight>> OPEN;

        std::unordered_set<Node> CLOSED;
        std::unordered_map<Node, Weight> distance_map;
        std::unordered_map<Node, Node*> parent_map;

        OPEN.push(new HeapNode<Node, Weight>(source_node, Weight{}));
        distance_map[source_node] = {};
        parent_map[source_node] = nullptr;

        while (!OPEN.empty()) {
            HeapNode<Node, Weight>* top_heap_node = OPEN.top();
            OPEN.pop();
            Node current_node = top_heap_node->getElement();
            delete top_heap_node;

            if (current_node == target_node) {
                // Found the path:
                cleanPriorityQueue<Node, Weight>(OPEN);
                Path<Node, Weight> path =
                    tracebackPath(
                        target_node,
                        parent_map,
                        weight_function);

                cleanParentMap<Node>(parent_map);
                return path;
            }

            if (CLOSED.contains(current_node)) {
                continue;
            }

            CLOSED.insert(current_node);

            const std::unordered_set<Node>* children =
                graph.getChildNodesOf(current_node);

            for (Node const& child_node : *children) {
                if (CLOSED.contains(child_node)) {
                    // The optimal distance from source_node to child 
                    // is known. Omit:
                    continue;
                }

                Weight distance =
                    distance_map[current_node] +
                    weight_function.getWeight(current_node, child_node);

                if (!parent_map.contains(child_node)
                    || distance < distance_map[child_node]) {
                    auto h =
                        heuristic_function->
                        estimate(child_node, target_node);

                    OPEN.push(
                        new HeapNode<Node, Weight>(
                            child_node,
                            distance + h));

                    distance_map[child_node] = distance;
                    Node* elem_ptr = new Node{ current_node };
                    parent_map[child_node] = elem_ptr;
                }
            }
        }

        throw PathDoesNotExistException{
            buildPathNotExistsErrorMessage(source_node, target_node) };
    }


} // End on namespace 'com::github::coderodde::pathfinders'.

#endif // COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_ASTAR_HPP