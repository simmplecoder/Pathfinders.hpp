#ifndef COM_GITHUB_CODERODDE_DIRECTED_GRAPH_HPP
#define COM_GITHUB_CODERODDE_DIRECTED_GRAPH_HPP

#include <cstddef> // for std::size_t
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <boost/json.hpp>

namespace com::github::coderodde::directed_graph {

    template<typename Node = int>
    class DirectedGraph {
    private:
        std::unordered_map<Node, std::unordered_set<Node>> child_map_;
        std::unordered_map<Node, std::unordered_set<Node>> parent_map_;
        std::unordered_set<Node> nodes_;
        std::size_t number_of_arcs_;

    public:
        DirectedGraph() : number_of_arcs_{ 0 } {}

        bool addNode(Node const& node) {
            if (!hasNode(node)) {
                child_map_[node] = {};
                parent_map_[node] = {};
                nodes_.insert(node);
                return true;
            }

            return false;
        }

        bool hasNode(Node const& node) {
            return nodes_.contains(node);
        }

        bool removeNode(Node const& node) {
            if (!hasNode(node)) {
                return false;
            }

            number_of_arcs_ -=
                child_map_[node].size() +
                parent_map_[node].size();

            child_map_.erase(node);
            parent_map_.erase(node);
            nodes_.erase(node);
            return true;
        }

        bool addArc(Node const& tail, Node const& head) {
            bool state_changed = false;

            if (!hasNode(tail)) {
                addNode(tail);
                state_changed = true;
            }

            if (!hasNode(head)) {
                addNode(head);
                state_changed = true;
            }

            if (!child_map_[tail].contains(head)) {
                child_map_[tail].insert(head);
                state_changed = true;
            }

            if (!parent_map_[head].contains(tail)) {
                parent_map_[head].insert(tail);
                state_changed = true;
            }

            if (state_changed) {
                number_of_arcs_++;
            }

            return state_changed;
        }

        bool hasArc(Node const& tail, Node const& head) {
            if (!child_map_.contains(tail)) {
                return false;
            }

            return child_map_[tail].contains(head);
        }

        bool removeArc(Node const& tail, Node const& head) {
            if (!child_map_.contains(tail)) {
                return false;
            }

            if (!child_map_[tail].contains(head)) {
                return false;
            }

            child_map_[tail].erase(head);
            parent_map_[head].erase(tail);
            number_of_arcs_--;
            return true;
        }

        std::unordered_set<Node>* getParentNodesOf(Node const& node) {
            return &parent_map_[node];
        }

        std::unordered_set<Node>* getChildNodesOf(Node const& node) {
            return &child_map_[node];
        }

        std::unordered_set<Node> const& getNodes() const noexcept {
            return nodes_;
        }

        [[nodiscard]] std::size_t getNumberOfNodes() const noexcept {
            return nodes_.size();
        }

        [[nodiscard]] std::size_t getNumberOfArcs() const noexcept {
            return number_of_arcs_;
        }

        [[nodiscard]] boost::json::object toJSON() const {
            boost::json::object result;

            boost::json::array nodes_array;
            for (const auto& node: nodes_) {
                nodes_array.push_back(node);
            }

            result["nodes"] = std::move(nodes_array);

            boost::json::array arcs_array;
            for (const auto& [node, parents]: parent_map_) {
                for (const auto& parent: parents) {
                    boost::json::object arc_desc;
                    arc_desc["to"] = node;
                    arc_desc["from"] = parent;
                    arcs_array.push_back(arc_desc);
                }
            }

            result["arcs"] = arcs_array;
            return result;
        }
    };

    template<typename Node = int>
    std::string buildNonExistingArcErrorMessage(
        Node const& tail,
        Node const& head) {

        std::stringstream ss;
        ss << "The arc (" << tail << ", " << head << ") does not exist.";
        return ss.str();
    }

    class NonExistingArcException : public std::logic_error {
    public:
        NonExistingArcException(std::string const& err_msg)
            :
            std::logic_error{ err_msg }
        {}
    };

    template<typename Node = int, typename Weight = double>
    class DirectedGraphWeightFunction {
    private:
        std::unordered_map<Node, std::unordered_map<Node, Weight>> weight_map_;

    public:
        void addWeight(Node const& parent, Node const& child, Weight weight) {
            weight_map_[parent][child] = weight;
        }

        void removeWeight(Node const& parent, Node const& child) {
            if (!weight_map_.contains(parent)
                || !weight_map_[parent].contains(child)) {
                return;
            }

            weight_map_[parent].erase(child);
        }

        Weight getWeight(Node const& parent, Node const& child) {
            if (!weight_map_.contains(parent)
                || !weight_map_[parent].contains(child)) {
                throw NonExistingArcException{
                    buildNonExistingArcErrorMessage(parent, child)
                };
            }

            return weight_map_[parent][child];
        }

        boost::json::array toJSON() const {
            boost::json::array weights;
            for (const auto& [parent, arcs]: weight_map_) {
                for (const auto& [child, arc_weight]: arcs) {
                    boost::json::object arc;
                    arc["from"] = parent;
                    arc["to"] = child;
                    arc["weight"] = arc_weight;

                    weights.push_back(arc);
                }
            }

            return weights;
        }
    };
} // End of namespace com::github::coderodde::directed_graph.

#endif // COM_GITHUB_CODERODDE_DIRECTED_GRAPH_HPP