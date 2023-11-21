//
// Created by rrivas on 20/11/2023.
//

#ifndef PROG3_SEC2_2023_2_GRAFOS_GRAPH_H
#define PROG3_SEC2_2023_2_GRAFOS_GRAPH_H
#include <vector>
#include <forward_list>
#include <unordered_map>

template <typename KeyType, typename ValueType, typename WeightType>
class graph {
    using IndexType = size_t;
    using ItemType = std::pair<IndexType, ValueType>;
    using AdjancentType = std::pair<IndexType, WeightType>;
    std::unordered_map<KeyType, ItemType> vertices;
    std::vector<std::forward_list<AdjancentType>> adjacent_list;

public:
    graph() = default;
    bool push_vertex(KeyType key, ValueType value) {
        if (vertices.find(key) == std::end(vertices)) {
            vertices.try_emplace(key, size(adjacent_list), value);
            adjacent_list.emplace_back();
        }
    }
    bool push_edge(std::tuple<KeyType, KeyType, WeightType> edge)

};

#endif //PROG3_SEC2_2023_2_GRAFOS_GRAPH_H
