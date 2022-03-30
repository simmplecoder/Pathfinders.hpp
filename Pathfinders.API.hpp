#ifndef COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_API_HPP
#define COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_API_HPP

#include "include/pathfinders/directed_graph.hpp"
#include "Pathfinders.Astar.hpp"
#include "pathfinders/djikstra.hpp"
#include "Pathfinders.BiDijkstra.hpp"
#include "Pathfinders.NBAstar.hpp"
#include "Pathfinders.SharedUtils.hpp"

namespace com::github::coderodde::pathfinders::api {
    
    using namespace com::github::coderodde::directed_graph;
    using namespace com::github::coderodde::pathfinders::util;

    template<typename Node = int, typename Weight = double>
    class Algorithm_2_Selector {
    private:
        DirectedGraph<Node>& directed_graph_;
        DirectedGraphWeightFunction<Node, Weight>&
            directed_graph_weight_function_;

        HeuristicFunction<Node, Weight>* heuristic_function_;
        Node& source_node_;
        Node& target_node_;

    public:
        Algorithm_2_Selector(
            DirectedGraph<Node>& directed_graph,
            DirectedGraphWeightFunction<Node, Weight>&
            directed_graph_weight_function,
            HeuristicFunction<Node, Weight>&
            heuristic_function,
            Node& source_node,
            Node& target_node)
            :
            directed_graph_{ directed_graph },
            directed_graph_weight_function_{ directed_graph_weight_function },
            heuristic_function_{ &heuristic_function },
            source_node_{ source_node },
            target_node_{ target_node }
        {

        }

        Path<Node, Weight> usingAstar() {
            return runAstarAlgorithm(
                directed_graph_,
                directed_graph_weight_function_,
                heuristic_function_,
                source_node_,
                target_node_);
        }

        Path<Node, Weight> usingBidirectionalAstar() {
            return runBidirectionalAstarAlgorithm(
                directed_graph_,
                directed_graph_weight_function_,
                heuristic_function_,
                source_node_,
                target_node_);
        }
    };

    template<typename Node = int, typename Weight = double>
    class Algorithm_1_Selector {
    private:
        DirectedGraph<Node>& directed_graph_;
        DirectedGraphWeightFunction<Node, Weight>&
            directed_graph_weight_function_;
        Node& source_node_;
        Node& target_node_;

    public:
        Algorithm_1_Selector(
            DirectedGraph<Node>& directed_graph,
            DirectedGraphWeightFunction<Node, Weight>&
            directed_graph_weight_function,
            Node& source_node,
            Node& target_node)
            :
            directed_graph_{ directed_graph },
            directed_graph_weight_function_{ directed_graph_weight_function },
            source_node_{ source_node },
            target_node_{ target_node }
        {

        }

        Path<Node, Weight> usingDijkstra() {
            return runDijkstrasAlgorithm(
                directed_graph_,
                directed_graph_weight_function_,
                source_node_,
                target_node_);
        }

        Path<Node, Weight> usingBidirectionalDijkstra() {
            return runBidirectionalDijkstrasAlgorithm(
                directed_graph_,
                directed_graph_weight_function_,
                source_node_,
                target_node_);
        }

        Algorithm_2_Selector<Node, Weight>
            withHeuristicFunction(
                HeuristicFunction<Node, Weight>& heuristic_function) {

            return Algorithm_2_Selector<Node, Weight>(
                directed_graph_,
                directed_graph_weight_function_,
                heuristic_function,
                source_node_,
                target_node_);
        }
    };

    template<typename Node = int, typename Weight = double>
    class TargetNodeSelector {
    private:
        DirectedGraph<Node>& directed_graph_;
        DirectedGraphWeightFunction<Node, Weight>&
            directed_graph_weight_function_;
        Node& source_node_;

    public:
        TargetNodeSelector(
            DirectedGraph<Node>& directed_graph,
            DirectedGraphWeightFunction<Node, Weight>& 
                directed_graph_weight_function,
            Node& source_node)
            :
            directed_graph_{ directed_graph },
            directed_graph_weight_function_{ directed_graph_weight_function },
            source_node_{ source_node }
        {

        }

        Algorithm_1_Selector<Node, Weight> to(Node& target_node) {
            return Algorithm_1_Selector<Node, Weight>{
                directed_graph_,
                    directed_graph_weight_function_,
                    source_node_,
                    target_node
            };
        }
    };

    template<typename Node = int, typename Weight = double>
    class SourceNodeSelector {
    private:
        DirectedGraph<Node>& directed_graph_;
        DirectedGraphWeightFunction<Node, Weight>&
            directed_graph_weight_function_;

    public:
        SourceNodeSelector(
            DirectedGraph<Node>& directed_graph,
            DirectedGraphWeightFunction<Node, Weight>& 
                directed_graph_weight_function)
            :
            directed_graph_{ directed_graph },
            directed_graph_weight_function_{ directed_graph_weight_function }
        {

        }

        TargetNodeSelector<Node, Weight> from(Node& node) {
            return TargetNodeSelector{
                directed_graph_,
                directed_graph_weight_function_,
                node
            };
        }
    };

    template<typename Node = int, typename Weight = double>
    class WeightFunctionSelector {
    private:
        DirectedGraph<Node>& directed_graph_;

    public:
        WeightFunctionSelector(DirectedGraph<Node>& directed_graph)
            :
            directed_graph_{ directed_graph }
        {}

        SourceNodeSelector<Node, Weight> withWeights(
            DirectedGraphWeightFunction<Node, Weight>&
            directed_graph_weight_function) {
            return SourceNodeSelector(
                directed_graph_, 
                directed_graph_weight_function);
        }
    };

    template<typename Node = int, typename Weight = double>
    class GraphSelector {
    public:
        WeightFunctionSelector<Node, Weight> in(DirectedGraph<Node>& 
            directed_graph) {
            return WeightFunctionSelector<Node, Weight>(directed_graph);
        }
    };

    template<typename Node = int, typename Weight = double>
    GraphSelector<Node, Weight> findShortestPath() {    
        return GraphSelector<Node, Weight>{};
    }
} // End of namespace com::github::coderodde::pathfinders::api.

#endif // COM_GITHUB_CODERODDE_GRAPH_PATHFINDERS_API_HPP