#pragma once
#include "core/interfaces.hpp"
#include <sstream>
#include <unordered_map>
namespace core {
class Graph {
public:
    struct OwnedComponent {
        std::string name;
        std::unique_ptr<IComponent> component;
    };
    explicit Graph(std::vector<OwnedComponent> components) : components_(std::move(components)) {
        std::unordered_map<IBlock *, std::size_t> index;
        for (auto &component : components_)
            for (auto &block : component.component->blocks()) {
                index.emplace(block.block.get(), nodes_.size());
                nodes_.push_back({component.name + "/" + block.name, block.block.get()});
            }
        std::vector<std::size_t> degree(nodes_.size());
        std::vector<std::vector<std::size_t>> edges(nodes_.size());
        for (std::size_t i = 0; i < nodes_.size(); ++i)
            for (auto *port : nodes_[i].block->inputs()) {
                auto source = index.find(port->source());
                if (source == index.end())
                    throw std::invalid_argument("unconnected or foreign graph input: " + nodes_[i].name);
                connections_.push_back({source->second, i, port->delayed()});
                if (port->delayed())
                    delayed_.push_back(port);
                else {
                    edges[source->second].push_back(i);
                    ++degree[i];
                }
            }
        for (std::size_t i = 0; i < degree.size(); ++i)
            if (degree[i] == 0)
                order_.push_back(i);
        for (std::size_t cursor = 0; cursor < order_.size(); ++cursor)
            for (auto target : edges[order_[cursor]])
                if (--degree[target] == 0)
                    order_.push_back(target);
        if (order_.size() != nodes_.size())
            throw std::invalid_argument("graph contains an undelayed cycle");
        for (auto &node : nodes_)
            for (auto *port : node.block->inputs())
                port->prepare();
    }
    Graph(const Graph &) = delete;
    Graph &operator=(const Graph &) = delete;
    Graph(Graph &&) = default;
    Graph &operator=(Graph &&) = default;
    void evaluate(const Context &context) {
        for (auto index : order_)
            nodes_[index].block->evaluate(context);
        for (auto *port : delayed_)
            port->latch();
    }
    void reset() {
        for (auto &node : nodes_)
            node.block->reset();
    }
    [[nodiscard]] std::string dot() const {
        std::ostringstream out;
        out << "digraph pipeline {\n";
        std::size_t node = 0;
        for (std::size_t c = 0; c < components_.size(); ++c) {
            out << "  subgraph cluster_" << c << " { label=\"" << escaped(components_[c].name) << "\";\n";
            for (const auto &block : components_[c].component->blocks()) {
                out << "    n" << node++ << " [label=\"" << escaped(block.name) << "\"];\n";
            }
            out << "  }\n";
        }
        for (auto &edge : connections_)
            out << "  n" << edge.from << " -> n" << edge.to << (edge.delayed ? " [style=dashed,label=\"previous evaluation\"]" : "") << ";\n";
        return out.str() + "}\n";
    }

private:
    static std::string escaped(std::string_view value) {
        std::string result;
        for (char c : value) {
            if (c == '"' || c == '\\')
                result += '\\';
            if (c == '\n')
                result += "\\n";
            else
                result += c;
        }
        return result;
    }
    struct Node {
        std::string name;
        IBlock *block;
    };
    struct Connection {
        std::size_t from, to;
        bool delayed;
    };
    std::vector<OwnedComponent> components_;
    std::vector<Node> nodes_;
    std::vector<std::size_t> order_;
    std::vector<Connection> connections_;
    std::vector<IInPort *> delayed_;
};
class GraphBuilder {
public:
    template <class C, class... Args>
        requires std::derived_from<C, IComponent> && std::constructible_from<C, Args...>
    C &makeComponent(std::string name, Args &&...args) {
        auto component = std::make_unique<C>(std::forward<Args>(args)...);
        auto &result = *component;
        components_.push_back({std::move(name), std::move(component)});
        return result;
    }
    [[nodiscard]] Graph build() && { return Graph{std::move(components_)}; }

private:
    std::vector<Graph::OwnedComponent> components_;
};
} // namespace core
