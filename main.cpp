#include "include/pathfinders/directed_graph.hpp"
#include "pathfinders/djikstra.hpp"
//#include "Pathfinders.SharedUtils.hpp"
//#include "Pathfinders.API.hpp"
#include <chrono>
#include <cstddef>
#include <iostream>
#include <random>
#include <fmt/format.h>
#include <boost/json.hpp>
#include <CLI/CLI.hpp>

constexpr std::size_t NUMBER_OF_NODES = 100 * 1000;
constexpr std::size_t NUMBER_OF_ARCS = 500 * 1000;
constexpr double SPACE_WIDTH = 10000.0;
constexpr double SPACE_HEIGHT = 10000.0;
constexpr double DISTANCE_FACTOR = 1.1;
constexpr double MIN_WEIGHT = 1.0;
constexpr double MAX_WEIGHT = 100.0;



//using namespace com::github::coderodde::directed_graph;
//using namespace com::github::coderodde::pathfinders;
//using namespace com::github::coderodde::pathfinders::api;

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
//
//class GraphData {
//private:
//    DirectedGraph<int> graph_;
//    DirectedGraphWeightFunction<int, double> weight_function_;
//    MyHeuristicFunction heuristic_function_;
//
//public:
//    GraphData(
//        DirectedGraph<int> graph,
//        DirectedGraphWeightFunction<int, double> weight_function,
//        MyHeuristicFunction heuristic_function)
//        :
//        graph_{ graph },
//        weight_function_{ weight_function },
//        heuristic_function_{ heuristic_function }
//    {}
//
//    DirectedGraph<int>& getGraph() {
//        return graph_;
//    }
//
//    DirectedGraphWeightFunction<int, double>& getWeightFunction() {
//        return weight_function_;
//    }
//
//    HeuristicFunction<int, double>& getHeuristicFunction() {
//        return heuristic_function_;
//    }
//};

EuclideanCoordinates getRandomEuclideanCoordinates(
        std::mt19937& mt,
        std::uniform_real_distribution<double> x_coord_distribution,
        std::uniform_real_distribution<double> y_coord_distribution) {
    double x = x_coord_distribution(mt);
    double y = y_coord_distribution(mt);
    EuclideanCoordinates coords{ x, y };
    return coords;
}

struct graph_type {
    using vertex_set_t = pathfinders::frozen_vertex_set<int>;
    vertex_set_t vertices;
    pathfinders::directed_weighted_edge_set<vertex_set_t, double> weights;
};

graph_type createRandomGraphData(std::size_t number_of_nodes,
                                 std::size_t number_of_arcs) {
    const std::size_t random_seed = 38;
    std::mt19937 mt(random_seed);
    std::uniform_int_distribution<std::size_t>
        uniform_distribution(0, number_of_nodes - 1);

    std::uniform_real_distribution<double> weight_distribution(MIN_WEIGHT, MAX_WEIGHT);

    std::unordered_map<int, EuclideanCoordinates> coordinate_map;

    pathfinders::vertex_set<int> vertices;
    for (std::size_t node_id = 0; node_id < number_of_nodes; ++node_id) {
        vertices.insert(static_cast<int>(node_id));
    }

    auto frozen_vertices = pathfinders::frozen_vertex_set<int>(std::move(vertices));
    auto edges = pathfinders::directed_weighted_edge_set<decltype(frozen_vertices), double>(frozen_vertices);
    for (size_t i = 0; i < number_of_arcs; ++i) {
        std::size_t tail_index = uniform_distribution(mt);
        std::size_t head_index = uniform_distribution(mt);
        edges.add_edge(frozen_vertices.iterator_for(static_cast<int>(head_index)).value(),
                       frozen_vertices.iterator_for(static_cast<int>(tail_index)).value(),
                       weight_distribution(mt));
    }

    return graph_type {
        .vertices = std::move(frozen_vertices),
        .weights = std::move(edges)
    };
}

template <typename Node>
struct Timing {
    Node from;
    Node to;
    std::chrono::nanoseconds duration;

    boost::json::object toJSON() const {
        boost::json::object result;
        result["from"] = from;
        result["to"] = to;
        result["duration_ns"] = duration.count();

        return result;
    }
};

int main(int argc, char* argv[]) {
    CLI::App app{"benchmark showcase of the library"};
    std::string output;
    app.add_option("o", output, "output file path to write benchmark results to")
        ->required(true)
        ->check(CLI::NonexistentPath);

    CLI11_PARSE(app, argc, argv)

    graph_type graph_data = createRandomGraphData(NUMBER_OF_NODES,
                                                  NUMBER_OF_ARCS);

    const std::size_t seed = 40;
    std::mt19937 mt(seed);
    const unsigned int run_count = 64;
    std::vector<Timing<int>> timings;
    timings.reserve(run_count);

    auto all_start = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < run_count; ++i) {
        try {
            std::uniform_int_distribution<int> dist(0, NUMBER_OF_NODES - 1);

            int source_node = dist(mt);
            int target_node = dist(mt);

            auto start_time = std::chrono::steady_clock::now();
            pathfinders::directed_weighted_path<int, double> path;

            Path<int, double> path =
                    findShortestPath()
                            .in(graph_data.getGraph())
                            .withWeights(graph_data.getWeightFunction())
                            .from(source_node)
                            .to(target_node)
                            .usingDijkstra();

            auto end_time = std::chrono::steady_clock::now();

            timings.push_back({source_node,
                               target_node,
                               std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time)});
            fmt::print("path from {} to {} is {} units long\n", source_node, target_node, path.distance());
        }
        catch (NodeNotPresentInGraphException const &err) {
            std::cout << err.what() << "\n";
        }
        catch (PathDoesNotExistException const &err) {
            std::cout << err.what() << "\n";
        }
    }
    auto all_end = std::chrono::steady_clock::now();

    boost::json::object result;
    result["seed"] = seed;
    result["graph"] = graph_data.getGraph().toJSON();
    result["run_count"] = run_count;
    result["total_time_ns"] = std::chrono::duration_cast<std::chrono::nanoseconds>(all_end - all_start).count();

    boost::json::array timings_array;
    for (const auto& timing: timings) {
        timings_array.push_back(timing.toJSON());
    }
    result["timings"] = std::move(timings_array);

    auto result_str = boost::json::serialize(result);
    std::ofstream output_file(output);
    if (!output_file) {
        fmt::print(stderr, "failed to open output file, printing to STDOUT\n");
        std::cout << result_str << '\n';
    }

    output_file << result_str << '\n';
    std::cout << output << '\n';
}
