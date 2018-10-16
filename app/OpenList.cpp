// Copyright [2018] <Yu-Kai Wang>

#include "OpenList.h"
#include <algorithm>
#include <functional>

void OpenList::Insert(const double &new_key,
                      const std::pair<int, int> &new_node) {
    priority_queue.push_back(std::make_tuple(
                             new_key, new_node.first, new_node.second));
    std::push_heap(priority_queue.begin(),
                   priority_queue.end(), std::greater<>());
}

void OpenList::UpdateKey(const double &new_key,
                         const std::pair<int, int> &position) {
    for (auto &node : priority_queue) {
        if (std::get<1>(node) == position.first &&
            std::get<2>(node) == position.second) {
            std::get<0>(node) = new_key;
            break;
        }
    }
    std::make_heap(priority_queue.begin(),
                   priority_queue.end(), std::greater<>());
}  // could be optimized

void OpenList::Remove(const std::pair<int, int> &node) {
    double min_key = -1.0;
    UpdateKey(min_key, node);
    std::pop_heap(priority_queue.begin(),
                  priority_queue.end(), std::greater<>());
    priority_queue.pop_back();
}

std::pair<double, std::pair<int, int>> OpenList::Top() const {
    auto key = std::get<0>(priority_queue.front());
    auto position = std::make_pair(std::get<1>(priority_queue.front()),
                                   std::get<2>(priority_queue.front()));
    return std::make_pair(key, position);
}

std::pair<double, std::pair<int, int>> OpenList::Pop() {
    std::pop_heap(priority_queue.begin(),
                  priority_queue.end(), std::greater<>());
    auto top_node = priority_queue.back();
    priority_queue.pop_back();
    auto key = std::get<0>(top_node);
    auto position = std::make_pair(std::get<1>(top_node),
                                   std::get<2>(top_node));
    return std::make_pair(key, position);
}

bool OpenList::Find(const std::pair<int, int> &node_to_find) const {
    for (auto const &node : priority_queue) {
        if (std::get<1>(node) == node_to_find.first &&
            std::get<2>(node) == node_to_find.second)
            return true;
    }
    return false;
}
