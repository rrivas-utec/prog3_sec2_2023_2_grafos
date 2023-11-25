//
// Created by rrivas on 20/11/2023.
//

#ifndef PROG3_SEC2_2023_2_GRAFOS_GRAPH_H
#define PROG3_SEC2_2023_2_GRAFOS_GRAPH_H
#include <vector>
#include <forward_list>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <stack>
#include <queue>
#include <unordered_set>

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
        // Funcion privada get_key
        KeyType get_key(size_t index) {
            auto it = std::find_if(
                    begin(vertices), end(vertices),
                    [index](auto item) {
                       return item.second.first == index;
                    });
            if (it != end(vertices))
                return it->first;
            return KeyType{};
        }
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
            // Estructuras que se utilizar
            std::queue<size_t> q;
            std::unordered_set<size_t> visited;
            // agrega el valor seleccionada al queue y visited
            auto idx = vertices[key].first;
            q.push(idx);
            visited.insert(idx);
            while (!q.empty()) {
                idx = q.front();
                fn(get_key(idx));
                q.pop();
                for (const auto& adj: buckets[idx]) {
                    if (visited.insert(adj.first).second) {
                        q.push(adj.first);
                    }
                }
            }
        }
        template<typename UnaryFunction>
        void dfs(KeyType key, UnaryFunction fn) {
            // Estructuras
            std::stack<size_t> s;
            std::unordered_set<size_t> visited;
            // El vertice seleccionando
            auto idx = vertices[key].first;
            // Agregar el vertice seleccionado al stack y visited
            s.push(idx);
            visited.insert(idx);
            while (!s.empty()) {
                // Paso # 3
                auto actual = s.top();
                // Paso # 4
                auto adj_list = buckets[actual];
                // Buscar el primer no visitado
                auto it = std::find_if(
                        begin(adj_list), end(adj_list),
                        [&visited](const auto& adj) {
                            return !visited.contains(adj.first);
                        });
                if (it != std::end(adj_list)) { // No visitado
                    s.push(it->first);
                    visited.insert(it->first);
                } else {
                    fn(get_key(actual));
                    s.pop();
                }

            }
        }
    };
}

#endif //PROG3_SEC2_2023_2_GRAFOS_GRAPH_H
