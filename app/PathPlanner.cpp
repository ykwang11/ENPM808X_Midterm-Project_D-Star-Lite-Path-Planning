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
* @file PathPlanner.h
* @author Yu-Kai Wang
* @copyright MIT License
*
* @brief D* Lite Path Planning
*
* PathPlanner steers the robot to the goal based on D* Lite algorithm.
*
*/

#include "PathPlanner.h"
#include "Map.h"

/**
* @brief Initialize the map and the open list
* @param map_ptr the pointer of the map
* @param openlist_ptr the pointer of the open list
* @return void
*/
PathPlanner::PathPlanner(std::pair<int, int> robot_,
        std::pair<int, int> goal_,
        std::vector<std::pair<int, int>> obstacle_,
        std::vector<std::pair<int, int>> hidden_) {

    map.setObstacle(obstacle_, hidden_);
    map.setGoal(goal_);
    map.setStart(robot_);

    // One lookahead cost of the goal must be zero
    auto goal_rhs = 0.0;
    map.setRhs(map.getGoal(), goal_rhs);
    // Insert the goal to open list
    auto new_key = map.calculateKey(map.getGoal());
    openlist.insert(new_key, map.getGoal());
}

/**
* @brief Compute the shortest path
* @param robot the robot
* @param map_ptr the pointer of the map
* @param openlist_ptr the pointer of the open list
* @return void
*/
void PathPlanner::computeShortestPath(const std::pair<int, int> &robot) {

//std::cout << "key value of the robot's positon: " << map.calculateKey(robot.CurrentPosition()) << std::endl;	
bool inconsistent = map.getRhs(robot) != map.getG(robot);

while (openlist.top().first <
        map.calculateKey(robot) ||
        inconsistent) {
    auto key_and_node = openlist.pop();
    auto node = key_and_node.second;

    auto old_key = key_and_node.first;
    auto new_key = map.calculateKey(node);

    //std::cout << "top node's key: "<< key_and_node.first << std::endl;
    //std::cout << "top node's position: "<< node.first << "," << node.second << std::endl;

    if (old_key < new_key) {
        //std::cout << "ComputeShortestPath case1" << std::endl;
        openlist.insert(new_key, node);
        } else if (map.getG(node) > map.getRhs(node)) {
            //std::cout << "ComputeShortestPath case2" << std::endl;
        map.setG(node, map.getRhs(node));
            auto successors = map.findNeighbors(node);
            //std::cout << "successors: " << successors.size() << std::endl;
            for (auto const &vertex : successors) {
                updateVertex(vertex);
            }
        } else {
            //std::cout << "ComputeShortestPath case3" << std::endl;
            map.setInfiniteG(node);
            updateVertex(node);
            for (auto const &vertex : map.findNeighbors(node)) {
                updateVertex(vertex);
            }
        }
        inconsistent = map.getRhs(robot) != map.getG(robot);
    }
    // Show the new computed shortest path.
    map.printG();
    map.printRhs();
}

/**
* @brief Update node of interest
* @param vertex the position of the node
* @param map_ptr the pointer of the map
* @param openlist_ptr the pointer of the open list
* @return void
*/
void PathPlanner::updateVertex(const std::pair<int, int> &vertex) {
    if (vertex != map.getGoal()) {
        //std::cout << " UpdateVertex1" << std::endl;
        map.setRhs(vertex, getMinRhs(vertex));
    }
    if (openlist.find(vertex)) {
        //std::cout << " UpdateVertex2" << std::endl;
        openlist.remove(vertex);
    }
    if (map.getG(vertex) != map.getRhs(vertex)) {
        //std::cout << " UpdateVertex3" << std::endl;
        openlist.insert(map.calculateKey(vertex), vertex);
    }
}

/**
* @brief Find the numimum rhs of amoung node's neighbors.
* @param vertex the position of the node
* @param map_ptr the pointer of the map
* @return minimum rhs
*/
double PathPlanner::getMinRhs(const std::pair<int, int> &vertex) {
    double min_rhs = map.infinity_cost;
    auto neibors = map.findNeighbors(vertex);
    for (auto const &next_vertex : neibors) {
        auto temp_rhs = map.computeCost(vertex, next_vertex) +
            map.getG(next_vertex);
        if (temp_rhs < min_rhs) min_rhs = temp_rhs;
    }
    return min_rhs;
}

/**
* @brief Find next position with minimum g-value plus travel cost
* @param current_position the position of the current node
* @param map_ptr the pointer of the map
* @return next position in the shortest path
*/
std::pair<int, int> PathPlanner::getNextPotision(
    const std::pair<int, int> &current_position) {
    auto next_position = current_position;
    double cheapest_cost = map.infinity_cost;
    for (auto const &candidate : map.findNeighbors(current_position)) {
        auto cost = map.computeCost(current_position, candidate) +
            map.getG(candidate);
        if (cost < cheapest_cost) {
            cheapest_cost = cost;
            next_position = candidate;
        }
    }
	return next_position;
}

/**
* @brief Find hidden obstacle and recognize it a obstacle
* @param current_position robot's current position
* @param map_ptr the pointer of the map
* @param openlist_ptr the pointer of the open list
* @return if there are hidden obstacle around
*/
bool PathPlanner::detectHidden(const std::pair<int, int> &current_position) {
    auto is_changed = false;
    // detect neibors of the current position
    for (auto const &candidate : map.findNeighbors(current_position)) {
        if (map.getStatus(candidate) == -1) {
            map.setStatus(candidate, 100);
            is_changed = true;
            // Update node's status
            updateVertex(candidate);
            // neibors of the change
            for (auto const &candidate_neighbor :
                map.findNeighbors(candidate)) {
                updateVertex(candidate_neighbor);
            }
            map.setRhs(candidate, map.infinity_cost);
            map.setG(candidate, map.infinity_cost);
        }
    }
    //std::cout << "After detection: ";
    //map.printG();
    return is_changed;
}

void PathPlanner::setMapTrace(const std::pair<int, int> &trace) {
    map.setTrace(trace);
}

void PathPlanner::print(){
    map.printResult();
}
