/**
 * @file OpenList.cpp
 * @author Yu-Kai Wang
 * @copyright MIT License
 *
 * @brief D* Lite Path Planning
 *
 * @section Description
 *
 * This class saves candidate nodes to search in minimum heap data structure.
 * 
 */

#include "OpenList.h"
#include <algorithm>
#include <functional>

/**
 * @brief Inset a node in the open list.
 * @param a candidate node's priority in searching and its position
 * @return none
 */
void OpenList::Insert(const double &new_key,
                      const std::pair<int, int> &new_node) {
    priority_queue.push_back(std::make_tuple(
                             new_key, new_node.first, new_node.second));
    std::push_heap(priority_queue.begin(),
                   priority_queue.end(), std::greater<>());
}

/**
 * @brief Update the key of node in the open list.
 * @param a candidate node's new priority in searching and its position
 * @return none
 */
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
}

/**
 * @brief Remove a node from the open list.
 * @param a node's position
 * @return none
 */
void OpenList::Remove(const std::pair<int, int> &node) {
    double min_key = -1.0;
    UpdateKey(min_key, node);
    std::pop_heap(priority_queue.begin(),
                  priority_queue.end(), std::greater<>());
    priority_queue.pop_back();
}

/**
 * @brief Get the node on the top of the open list (a minimum heap).
 * @param none
 * @return the top node's priority in searching and its position
 */
std::pair<double, std::pair<int, int>> OpenList::Top() const {
    auto key = std::get<0>(priority_queue.front());
    auto position = std::make_pair(std::get<1>(priority_queue.front()),
                                   std::get<2>(priority_queue.front()));
    return std::make_pair(key, position);
}

/**
 * @brief Get the node on the top of the open list and romovee it.
 * @param none
 * @return the top node's priority in searching and its position
 */
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

/**
 * @brief Find if a node is in the open list.
 * @param none
 * @return true if the node exsit and false if not
 */
bool OpenList::Find(const std::pair<int, int> &node_to_find) const {
    for (auto const &node : priority_queue) {
        if (std::get<1>(node) == node_to_find.first &&
            std::get<2>(node) == node_to_find.second)
            return true;
    }
    return false;
}
