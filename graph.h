//
// Created by rrivas on 20/11/2023.
//

#ifndef PROG3_SEC2_2023_2_GRAFOS_GRAPH_H
#define PROG3_SEC2_2023_2_GRAFOS_GRAPH_H
#include <vector>
#include <forward_list>
#include <unordered_map>
#include <limits>

namespace utec {
    template<typename KeyType, typename ValueType, typename WeightType>
    class graph {
        // implemento los tipos
        using ItemType = std::pair<size_t, ValueType>;
        using AdjacentType = std::pair<size_t, WeightType>;
        using AdjacentListType = std::forward_list<AdjacentType>;
        // Implementación de atributos
        std::unordered_map<KeyType, ItemType> vertices;
        std::vector<AdjacentListType> buckets;
    public:
        graph() = default;

        bool push_vertex(KeyType key, ValueType value) {
//            if (vertices.insert({key, {std::size(buckets), value}}).second) {
//                buckets.emplace();
//            }
            if (vertices.try_emplace(key, std::size(buckets), value).second) {
                buckets.emplace_back();
                return true;
            }
            return false;
        }

        void push_edge(std::tuple<KeyType, KeyType, WeightType> edge) {
//            const auto& [u, v, w] = edge;
            const auto& u = std::get<0>(edge);
            const auto& v = std::get<1>(edge);
            const auto& w = std::get<2>(edge);
            // Validación de los vertices
            if (!vertices.contains(u)) throw std::runtime_error("Error");
            if (!vertices.contains(v)) throw std::runtime_error("Error");
            // Buscamos los indices
            auto idx_u = vertices[u].first;
            auto idx_v = vertices[v].first;
            // Insertar un nuevo adjacente
            buckets[idx_u].emplace_front(idx_v, w);
            buckets[idx_v].emplace_front(idx_u, w);
        }

        ValueType get_vertex(KeyType key) {
            if (!vertices.contains(key)) throw std::runtime_error("Error");
            return vertices[key].second;
        }

        WeightType get_edge(KeyType u, KeyType v) {
            // Validacion
            if (!vertices.contains(u)) throw std::runtime_error("Error");
            if (!vertices.contains(v)) throw std::runtime_error("Error");
            // Buscamos los indices
            auto idx_u = vertices[u].first;
            auto idx_v = vertices[v].first;
            auto it = std::find_if(
                    std::begin(buckets[idx_u]), std::end(buckets[idx_u]),
                [idx_v](auto adj) { return adj.first == idx_v; });
            if (it != std::end(buckets[idx_u])) {
                return it->second;
            }
            else {
                return std::numeric_limits<WeightType>::infinity();
            }
        }

        template<typename UnaryFunction>
        void bfs(KeyType key, UnaryFunction fn) {

        }

    };
}

#endif //PROG3_SEC2_2023_2_GRAFOS_GRAPH_H
