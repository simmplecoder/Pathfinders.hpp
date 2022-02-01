#ifndef COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_BI_DIJKSTRA_HPP
#define COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_BI_DIJKSTRA_HPP

#include "DirectedGraph.hpp"
#include "Pathfinders.SharedUtils.hpp"
#include <iostream>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace com::github::coderodde::pathfinders {

	using namespace com::github::coderodde::directed_graph;
	using namespace com::github::coderodde::pathfinders::util;

	template<typename Node = int, typename Weight = double>
	Path<Node, Weight>
		runBidirectionalDijkstrasAlgorithm(
			DirectedGraph<Node>& graph,
			DirectedGraphWeightFunction<Node, Weight>& weight_function,
			Node& source_node,
			Node& target_node) {
		checkTerminalNodes(graph, source_node, target_node);

		std::priority_queue <
			HeapNode<Node, Weight>*,
			std::vector<HeapNode<Node, Weight>*>,
			HeapNodeComparator<Node, Weight>> OPEN_FORWARD;

		std::priority_queue <
			HeapNode<Node, Weight>*,
			std::vector<HeapNode<Node, Weight>*>,
			HeapNodeComparator<Node, Weight>> OPEN_BACKWARD;

		std::unordered_set<Node> CLOSED_FORWARD;
		std::unordered_set<Node> CLOSED_BACKWARD;

		std::unordered_map<Node, Weight> distance_map_forward;
		std::unordered_map<Node, Weight> distance_map_backward;

		std::unordered_map<Node, Node*> parent_map_forward;
		std::unordered_map<Node, Node*> parent_map_backward;

		OPEN_FORWARD.push(new HeapNode<Node, Weight>(source_node, Weight{}));
		OPEN_BACKWARD.push(new HeapNode<Node, Weight>(target_node, Weight{}));

		distance_map_forward[source_node] = Weight{};
		distance_map_backward[target_node] = Weight{};

		parent_map_forward[source_node] = nullptr;
		parent_map_backward[target_node] = nullptr;

		const Node* touch_node = nullptr;
		Weight best_cost = std::numeric_limits<Weight>::max();

		while (!OPEN_FORWARD.empty() && !OPEN_BACKWARD.empty()) {
			if (OPEN_FORWARD.top()->getDistance() +
				OPEN_BACKWARD.top()->getDistance() >= best_cost) {

				cleanPriorityQueue(OPEN_FORWARD);
				cleanPriorityQueue(OPEN_BACKWARD);

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

			if (OPEN_FORWARD.size() + CLOSED_FORWARD.size() <
				OPEN_BACKWARD.size() + CLOSED_BACKWARD.size()) {
				HeapNode<Node, Weight>* top_heap_node = OPEN_FORWARD.top();
				OPEN_FORWARD.pop();
				Node current_node = top_heap_node->getElement();
				delete top_heap_node;

				CLOSED_FORWARD.insert(current_node);

				const std::unordered_set<Node>* children =
					graph.getChildNodesOf(current_node);

				for (Node const& child_node : *children) {
					if (CLOSED_FORWARD.contains(child_node)) {
						// The optimal distance from source_node to
						// child_node is known. Omit:
						continue;
					}

					Weight distance =
						distance_map_forward[current_node] +
						weight_function.getWeight(current_node, child_node);

					if (!distance_map_forward.contains(child_node)
						|| distance_map_forward[child_node] > distance) {
						OPEN_FORWARD.push(
							new HeapNode<Node, Weight>(
								child_node,
								distance));

						distance_map_forward[child_node] = distance;
						Node* node_ptr = new Node{ current_node };
						parent_map_forward[child_node] = node_ptr;

						if (CLOSED_BACKWARD.contains(child_node)) {
							Weight path_length =
								distance + distance_map_backward[child_node];

							if (best_cost > path_length) {
								best_cost = path_length;
								touch_node = &child_node;
							}
						}
					}
				}
			}
			else {
				HeapNode<Node, Weight>* top_heap_node = OPEN_BACKWARD.top();
				OPEN_BACKWARD.pop();
				Node current_node = top_heap_node->getElement();
				delete top_heap_node;

				CLOSED_BACKWARD.insert(current_node);

				const std::unordered_set<Node>* parents =
					graph.getParentNodesOf(current_node);

				for (Node const& parent_node : *parents) {
					if (CLOSED_BACKWARD.contains(parent_node)) {
						// The optimal distance from parent_node to 
						// target_node is known. Omit:
						continue;
					}

					Weight distance =
						distance_map_backward[current_node] +
						weight_function.getWeight(parent_node, current_node);

					if (!distance_map_backward.contains(parent_node)
						|| distance_map_backward[parent_node] > distance) {
						OPEN_BACKWARD.push(new HeapNode<Node, Weight>(parent_node, distance));
						distance_map_backward[parent_node] = distance;
						Node* node_ptr = new Node{ current_node };
						parent_map_backward[parent_node] = node_ptr;

						if (CLOSED_FORWARD.contains(parent_node)) {
							Weight path_length =
								distance + distance_map_forward[parent_node];

							if (best_cost > path_length) {
								best_cost = path_length;
								touch_node = &parent_node;
							}
						}
					}
				}

			}
		}

		throw PathDoesNotExistException{
			buildPathNotExistsErrorMessage(source_node, target_node)
		};
	}
}

#endif // COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_BI_DIJKSTRA_HPP