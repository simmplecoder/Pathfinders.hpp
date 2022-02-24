#include "DirectedGraph.hpp"
#include "Pathfinders.SharedUtils.hpp"
#include "Pathfinders.API.hpp"
#include <chrono>
#include <cstddef>
#include <iostream>
#include <random>

constexpr std::size_t NUMBER_OF_NODES = 100 * 1000;
constexpr std::size_t NUMBER_OF_ARCS = 500 * 1000;
constexpr double SPACE_WIDTH = 10000.0;
constexpr double SPACE_HEIGHT = 10000.0;
constexpr double DISTANCE_FACTOR = 1.1;

using namespace com::github::coderodde::directed_graph;
using namespace com::github::coderodde::pathfinders;
using namespace com::github::coderodde::pathfinders::api;

class EuclideanCoordinates {
private:
    double x_;
    double y_;

public:
    EuclideanCoordinates(double x = 0.0, double y = 0.0) :
        x_{ x },
        y_{ y }
    {}

    double distanceTo(EuclideanCoordinates const& other) const {
        const auto dx = x_ - other.x_;
        const auto dy = y_ - other.y_;
        return std::sqrt(dx * dx + dy * dy);
    }
};

class MyHeuristicFunction : public HeuristicFunction<int, double> {
private:
    std::unordered_map<int, EuclideanCoordinates> map_;

public:
    MyHeuristicFunction(
        std::unordered_map<int, EuclideanCoordinates> map)
        : map_{ map } {}

    MyHeuristicFunction(const MyHeuristicFunction& other)
        :
        map_{ other.map_ }
    {

    }

    MyHeuristicFunction(MyHeuristicFunction&& other) {
        map_ = std::move(other.map_);
    }

    MyHeuristicFunction& operator=(const MyHeuristicFunction& other) {
        map_ = other.map_;
        return *this;
    }

    MyHeuristicFunction& operator=(MyHeuristicFunction&& other) {
        map_ = std::move(other.map_);
        return *this;
    }

    ~MyHeuristicFunction() {

    }

    double estimate(int const& tail, int const& head) override {
        const auto point1 = map_[tail];
        const auto point2 = map_[head];
        return point1.distanceTo(point2);
    }
};

class GraphData {
private:
    DirectedGraph<int> graph_;
    DirectedGraphWeightFunction<int, double> weight_function_;
    MyHeuristicFunction heuristic_function_;

public:
    GraphData(
        DirectedGraph<int> graph,
        DirectedGraphWeightFunction<int, double> weight_function,
        MyHeuristicFunction heuristic_function)
        :
        graph_{ graph },
        weight_function_{ weight_function },
        heuristic_function_{ heuristic_function }
    {}

    DirectedGraph<int>& getGraph() {
        return graph_;
    }

    DirectedGraphWeightFunction<int, double>& getWeightFunction() {
        return weight_function_;
    }

    HeuristicFunction<int, double>& getHeuristicFunction() {
        return heuristic_function_;
    }
};

EuclideanCoordinates getRandomEuclideanCoordinates(
    std::mt19937& mt,
    std::uniform_real_distribution<double> x_coord_distribution,
    std::uniform_real_distribution<double> y_coord_distribution) {
    double x = x_coord_distribution(mt);
    double y = y_coord_distribution(mt);
    EuclideanCoordinates coords{ x, y };
    return coords;
}

GraphData createRandomGraphData(std::size_t number_of_nodes,
    std::size_t number_of_arcs) {
    DirectedGraph<int> graph;
    DirectedGraphWeightFunction<int, double> weight_function;
    std::vector<int> node_vector;
    node_vector.reserve(number_of_nodes);
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<std::size_t>
        uniform_distribution(0, number_of_nodes - 1);

    std::uniform_real_distribution<double> x_coord_distribution(0, SPACE_WIDTH);
    std::uniform_real_distribution<double> y_coord_distribution(0, SPACE_HEIGHT);

    std::unordered_map<int, EuclideanCoordinates> coordinate_map;

    for (size_t node_id = 0; node_id < number_of_nodes; ++node_id) {
        graph.addNode((int)node_id);
        node_vector.push_back((int)node_id);
        EuclideanCoordinates coords =
            getRandomEuclideanCoordinates(
                mt,
                x_coord_distribution,
                y_coord_distribution);

        coordinate_map[(int)node_id] = coords;
    }

    for (size_t i = 0; i < number_of_arcs; ++i) {
        std::size_t tail_index = uniform_distribution(mt);
        std::size_t head_index = uniform_distribution(mt);
        int tail = node_vector[tail_index];
        int head = node_vector[head_index];
        EuclideanCoordinates tail_coords = coordinate_map[tail];
        EuclideanCoordinates head_coords = coordinate_map[head];
        graph.addArc(tail, head);
        weight_function.addWeight(tail,
            head,
            tail_coords.distanceTo(head_coords)
            * DISTANCE_FACTOR);
    }

    MyHeuristicFunction heuristic_function{ coordinate_map };
    GraphData graph_data(
        graph,
        weight_function,
        heuristic_function);

    return graph_data;
}

class Milliseconds {
private:
    std::chrono::high_resolution_clock m_clock;

public:
    auto milliseconds() {
        return std::chrono::duration_cast<std::chrono::milliseconds>
            (m_clock.now().time_since_epoch()).count();
    }
};

int main() {
    GraphData graph_data = createRandomGraphData(NUMBER_OF_NODES,
        NUMBER_OF_ARCS);

    try {
        Milliseconds ms;
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_int_distribution<int> dist(0, NUMBER_OF_NODES - 1);

        int source_node = dist(mt);
        int target_node = dist(mt);

        std::cout << "Source node: " << source_node << "\n";
        std::cout << "Target node: " << target_node << "\n";
        std::cout << "--- Dijkstra's algorithm: ---\n";

        auto start_time = ms.milliseconds();

        Path<int, double> path =
            findShortestPath()
            .in(graph_data.getGraph())
            .withWeights(graph_data.getWeightFunction())
            .from(source_node)
            .to(target_node)
            .usingDijkstra();

        auto end_time = ms.milliseconds();

        std::cout << "Path:\n";

        for (size_t i = 0; i < path.length(); ++i) {
            std::cout << path[i] << "\n";
        }

        std::cout << "Path distance: " << path.distance() << "\n";
        std::cout << "Duration: " << (end_time - start_time) << " ms.\n\n";
        std::cout << "--- Bidirectional Dijkstra's algorithm: ---\n";

        start_time = ms.milliseconds();

        path =
            findShortestPath()
            .in(graph_data.getGraph())
            .withWeights(graph_data.getWeightFunction())
            .from(source_node)
            .to(target_node)
            .usingBidirectionalDijkstra();

        end_time = ms.milliseconds();

        std::cout << "Path:\n";

        for (size_t i = 0; i < path.length(); ++i) {
            std::cout << path[i] << "\n";
        }

        std::cout << "Path distance: " << path.distance() << "\n";
        std::cout << "Duration: " << (end_time - start_time) << " ms.\n\n";
        std::cout << "--- A* algorithm: ---\n";

        start_time = ms.milliseconds();

        path =
            findShortestPath()
            .in(graph_data.getGraph())
            .withWeights(graph_data.getWeightFunction())
            .from(source_node)
            .to(target_node)
            .withHeuristicFunction(graph_data.getHeuristicFunction())
            .usingAstar();

        end_time = ms.milliseconds();

        std::cout << "Path:\n";

        for (size_t i = 0; i < path.length(); ++i) {
            std::cout << path[i] << "\n";
        }

        std::cout << "Path distance: " << path.distance() << "\n";
        std::cout << "Duration: " << (end_time - start_time) << " ms.\n\n";

        //// NBA* ///////////////////////////////////////////////////////////// 
        std::cout << "--- Bidirectional A* (NBA*) algorithm: ---\n";
        start_time = ms.milliseconds();

        path =
            findShortestPath()
            .in(graph_data.getGraph())
            .withWeights(graph_data.getWeightFunction())
            .from(source_node)
            .to(target_node)
            .withHeuristicFunction(graph_data.getHeuristicFunction())
            .usingBidirectionalAstar();

        end_time = ms.milliseconds();

        std::cout << "Path:\n";

        for (size_t i = 0; i < path.length(); ++i) {
            std::cout << path[i] << "\n";
        }

        std::cout << "Path distance: " << path.distance() << "\n";
        std::cout << "Duration: " << (end_time - start_time) << " ms.\n\n";
    }
    catch (NodeNotPresentInGraphException const& err) {
        std::cout << err.what() << "\n";
    }
    catch (PathDoesNotExistException const& err) {
        std::cout << err.what() << "\n";
    }
    /*
    DirectedGraph g2;
    DirectedGraphWeightFunction w2;

    g2.addNode(1);
    g2.addNode(2);
    g2.addNode(3);

    g2.addArc(1, 2);
    g2.addArc(2, 3);

    w2.addWeight(1, 2, 1.0);
    w2.addWeight(2, 3, 2.0);

    std::unordered_map<int, EuclideanCoordinates> coordinate_map;
    coordinate_map[1] = EuclideanCoordinates(0, 0);
    coordinate_map[2] = EuclideanCoordinates(0.5, 0.5);
    coordinate_map[3] = EuclideanCoordinates(1, 1);

    MyHeuristicFunction hf2(coordinate_map);
    int source_node = 1;
    int target_node = 3;
    runBidirectionalAstarAlgorithm<int, double>(
        g2, 
        w2, 
        &hf2, 
        source_node, 
        target_node);
        */
    return 0;
}