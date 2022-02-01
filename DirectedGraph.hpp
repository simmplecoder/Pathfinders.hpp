#ifndef COM_GITHUB_CODERODDE_DIRECTED_GRAPH_HPP
#define COM_GITHUB_CODERODDE_DIRECTED_GRAPH_HPP

#include <cstddef> // for std::size_t
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

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

        std::unordered_set<Node> const& getNodes() const {
            return nodes_;
        }

        std::size_t getNumberOfNodes() const {
            return nodes_.size();
        }

        std::size_t getNumberOfArcs() const {
            return number_of_arcs_;
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
        void addWeight(Node const& tail, Node const& head, Weight weight) {
            weight_map_[tail][head] = weight;
        }

        void removeWeight(Node const& tail, Node const& head) {
            if (!weight_map_.contains(tail) 
                || !weight_map_[tail].contains(head)) {
                return;
            }

            weight_map_[tail].erase(head);
        }

        Weight getWeight(Node const& tail, Node const& head) {
            if (!weight_map_.contains(tail)
                || !weight_map_[tail].contains(head)) {
                throw NonExistingArcException{
                    buildNonExistingArcErrorMessage(tail, head)
                };
            }

            return weight_map_[tail][head];
        }
    };
} // End of namespace com::github::coderodde::directed_graph.

#endif // COM_GITHUB_CODERODDE_DIRECTED_GRAPH_HPP