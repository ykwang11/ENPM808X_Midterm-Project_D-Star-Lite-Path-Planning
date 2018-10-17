/**
 * @file Map.cpp
 * @author Yu-Kai Wang
 * @copyright MIT License
 *
 * @brief D* Lite Path Planning
 *
 * @section Description
 *
 * This program plans the path of the robot, re-plan new path when the map 
 * changes and moves the robot to avoid obstacles. That is, it performs the 
 * D* Lite algorithm.
 * 
 */

#include <iostream>
#include <vector>
#include <utility>
#include <algorithm>

#include "Robot.h"
#include "Map.h"
#include "OpenList.h"

void Initialize(Map *, OpenList *);
void ComputeShortestPath(const Robot &, Map *, OpenList *);
void UpdateVertex(const std::pair<int, int> &, Map *, OpenList *);
double ComputeMinRhs(const std::pair<int, int> &, Map *);
std::pair<int, int> ComputeNextPotision(const std::pair<int, int> &, Map *);
bool DetectHiddenObstacle(const std::pair<int, int> &, Map *, OpenList *);

int main() {
    // Declaration
    Robot robot(std::make_pair(2, 4));
    Map map(4, 5);
    std::vector<std::pair<int, int>> obstacle, hidden_obstacle;
    OpenList openlist;

    // Setting the environment: obstacles. hedden obstacles, the goal, the robot
    obstacle.push_back(std::make_pair(1, 1));
    obstacle.push_back(std::make_pair(0, 2));
    obstacle.push_back(std::make_pair(1, 2));
    hidden_obstacle.push_back(std::make_pair(2, 2));
    map.AddObstacle(obstacle, hidden_obstacle);
    map.SetGoal(std::make_pair(0, 0));
    map.UpdateCellStatus(robot.CurrentPosition(), map.start_mark);

    // Initialize
    Initialize(&map, &openlist);

    // Compute shortest path in the beginning
    ComputeShortestPath(robot, &map, &openlist);

    // Keep moving until reach the goal
    while (robot.CurrentPosition() != map.GetGoal()) {
        auto next_position = ComputeNextPotision(robot.CurrentPosition(), &map);
        robot.Move(next_position);
        map.UpdateCellStatus(robot.CurrentPosition(), map.robot_mark);

        // Print out every step in the journey
        map.PrintResult();

        // Detect environmental change
        auto graph_changed =
             DetectHiddenObstacle(robot.CurrentPosition(), &map, &openlist);

        // Only re-plan path when robot detects change in the environment.
        if (graph_changed) ComputeShortestPath(robot, &map, &openlist);
    }

    std::cout << "Achieved!";
    return 0;
}

/**
 * @brief Initialize the map and the open list
 * @param map_ptr the pointer of the map
 * @param openlist_ptr the pointer of the open list
 * @return none
 */
void Initialize(Map* map_ptr, OpenList* openlist_ptr) {
    // One lookahead cost of the goal must be zero
    auto goal_rhs = 0.0;
    map_ptr->UpdateCellRhs(map_ptr->GetGoal(), goal_rhs);
    // Insert the goal to open list
    auto new_key = map_ptr->CalculateCellKey(map_ptr->GetGoal());
    openlist_ptr->Insert(new_key, map_ptr->GetGoal());
}

/**
 * @brief Compute the shortest path
 * @param robot the robot
 * @param map_ptr the pointer of the map
 * @param openlist_ptr the pointer of the open list
 * @return none
 */
void ComputeShortestPath(const Robot& robot,
                         Map* map_ptr, OpenList* openlist_ptr) {
    while (openlist_ptr->Top().first <
           map_ptr->CalculateCellKey(robot.CurrentPosition()) ||
           map_ptr->CurrentCellRhs(robot.CurrentPosition()) !=
           map_ptr->CurrentCellG(robot.CurrentPosition())) {
        auto key_and_node = openlist_ptr->Pop();
        auto node = key_and_node.second;

        auto old_key = key_and_node.first;
        auto new_key = map_ptr->CalculateCellKey(node);

        if (old_key < new_key) {
            openlist_ptr->Insert(new_key, node);
        } else if (map_ptr->CurrentCellG(node) >
                   map_ptr->CurrentCellRhs(node)) {
            map_ptr->UpdateCellG(node, map_ptr->CurrentCellRhs(node));
            for (auto const &vertex : map_ptr->FindNeighbors(node)) {
                UpdateVertex(vertex, map_ptr, openlist_ptr);
            }
        } else {
            map_ptr->SetInfiityCellG(node);
            UpdateVertex(node, map_ptr, openlist_ptr);
            for (auto const &vertex : map_ptr->FindNeighbors(node)) {
                UpdateVertex(vertex, map_ptr, openlist_ptr);
            }
        }
    }
    // Show the new computed shortest path.
    map_ptr->PrintValue();
    map_ptr->PrintResult();
}

/**
 * @brief Update node of interest
 * @param vertex the position of the node
 * @param map_ptr the pointer of the map
 * @param openlist_ptr the pointer of the open list
 * @return none
 */
void UpdateVertex(const std::pair<int, int> &vertex,
                  Map *map_ptr, OpenList *openlist_ptr) {
    if (vertex != map_ptr->GetGoal()) {
        map_ptr->UpdateCellRhs(vertex, ComputeMinRhs(vertex, map_ptr));
    }
    if (openlist_ptr->Find(vertex)) {
        openlist_ptr->Remove(vertex);
    }
    if (map_ptr->CurrentCellG(vertex) != map_ptr->CurrentCellRhs(vertex)) {
        openlist_ptr->Insert(map_ptr->CalculateCellKey(vertex), vertex);
    }
}

/**
 * @brief Find the numimum rhs of amoung node's neighbors.
 * @param vertex the position of the node
 * @param map_ptr the pointer of the map
 * @return minimum rhs 
 */
double ComputeMinRhs(const std::pair<int, int> &vertex, Map *map_ptr) {
    double min_rhs = map_ptr->infinity_cost;
    auto neibors = map_ptr->FindNeighbors(vertex);
    for (auto const &next_vertex : neibors) {
        auto temp_rhs = map_ptr->ComputeCost(vertex, next_vertex) +
                        map_ptr->CurrentCellG(next_vertex);
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
std::pair<int, int> ComputeNextPotision(
                    const std::pair<int, int> &current_position, Map *map_ptr) {
    auto next_position = current_position;
    double cheaest_cost = map_ptr->infinity_cost;
    for (auto const &candidate : map_ptr->FindNeighbors(current_position)) {
        auto cost = map_ptr->ComputeCost(current_position, candidate) +
                    map_ptr->CurrentCellG(candidate);
        if (cost < cheaest_cost) {
            cheaest_cost = cost;
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
bool DetectHiddenObstacle(const std::pair<int, int> &current_position,
                          Map *map_ptr, OpenList *openlist_ptr) {
    auto is_changed = false;
    for (auto const &candidate : map_ptr->FindNeighbors(current_position)) {
        if (map_ptr->CurrentCellStatus(candidate) == map_ptr->unknown_mark) {
            map_ptr->UpdateCellStatus(candidate, map_ptr->obstacle_mark);

            is_changed = true;
            // Update node's status
            UpdateVertex(candidate, map_ptr, openlist_ptr);

            for (auto const &candidate_neighbor :
                             map_ptr->FindNeighbors(candidate)) {
                UpdateVertex(candidate_neighbor, map_ptr, openlist_ptr);
            }
            map_ptr->UpdateCellRhs(candidate, map_ptr->infinity_cost);
            map_ptr->UpdateCellG(candidate, map_ptr->infinity_cost);
        }
    }
    return is_changed;
}

