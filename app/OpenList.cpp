/**
 * MIT Licence
 * Copyright (c) 2018 Yu-Kai Wang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */

/**
 * @file OpenList.cpp
 * @author Yu-Kai Wang
 * @copyright MIT License
 *
 * @brief D* Lite Path Planning
 *
 * This class saves candidate nodes to search in minimum heap data structure.
 * 
 */

#include "OpenList.h"
#include <algorithm>
#include <functional>

/**
 * @brief Inset a node in the open list.
 * @param new_key thepriority of the node to be added
 * @param new_node a candidate node's priority in searching and its position
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
 * @param new_key thepriority of the node to be changed
 * @param position a candidate node's new priority in searching and its position
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
 * @param node a node's position
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
