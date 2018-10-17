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
 * @file Map.cpp
 * @author Yu-Kai Wang
 * @copyright MIT License
 *
 * @brief D* Lite Path Planning
 *
 * @section Description
 *
 * This class controls cells in the map and provides all infomation and 
 * all methods that related to the environment.
 * 
 */

#include "Map.h"
#include <algorithm>

/**
 * @brief Constructor.
 * @param height the size of the map
 * @param width the size of the map
 * @return none
 */
Map::Map(const int &height, const int &width) {
    std::vector<std::vector<Cell>> new_grid(
        height,
        std::vector<Cell>(width, Cell(infinity_cost)));
    grid = new_grid;
    map_size = std::make_pair(height, width);
}

/**
 * @brief Add obstacles and change cells's status.
 * @param obstacle a set of obstackes's position
 * @param hidden_obstacle a set of hidden obstackes's position
 * @return none
 */
void Map::AddObstacle(const std::vector<std::pair<int, int>> &obstacle,
                      const std::vector<std::pair<int, int>> &hidden_obstacle) {
    for (auto const &node : obstacle)
        grid.at(node.first).at(node.second).UpdateStatus(obstacle_mark);
    for (auto const &node : hidden_obstacle)
        grid.at(node.first).at(node.second).UpdateStatus(unknown_mark);
}

/**
 * @brief Set the goal and change cells's status.
 * @param new_goal the position of the goal
 * @return none
 */
void Map::SetGoal(const std::pair<int, int> &new_goal) {
    goal = new_goal;
    UpdateCellStatus(new_goal, goal_mark);
}

/**
 * @brief Get the goal's position.
 * @return the position of the goal
 */
std::pair<int, int> Map::GetGoal() const { return goal; }

/**
 * @brief Get the g-value of the cell with given position.
 * @param position the position of of the cell
 * @return cell's g-value
 */
double Map::CurrentCellG(const std::pair<int, int> &position) const {
    return grid.at(position.first).at(position.second).CurrentG();
}

/**
 * @brief Get the rhs-value of the cell with given position.
 * @param position the position of of the cell
 * @return cell's rhs-value
 */
double Map::CurrentCellRhs(const std::pair<int, int> &position) const {
    return grid.at(position.first).at(position.second).CurrentRhs();
}

/**
 * @brief Caculat the key (priority in search) of the cell with given position.
 * @param position the position of of the cell
 * @return the key value, which is the priority in next search
 */
double Map::CalculateCellKey(const std::pair<int, int> &position) const {
    return std::min(CurrentCellG(position), CurrentCellRhs(position));
}

/**
 * @brief Get the status of the cell with given position.
 * @param position the position of of the cell
 * @return cell's status
 */
std::string Map::CurrentCellStatus(const std::pair<int, int> &position) const {
    return grid.at(position.first).at(position.second).CurrentStatus();
}

/**
 * @brief Set the g-value of the cell with given position.
 * @param position the position of of the cell
 * @param new_g new estamated distance to the goal
 * @return none
 */
void Map::UpdateCellG(const std::pair<int, int> &position,
                      const double &new_g) {
    grid.at(position.first).at(position.second).UpdateG(new_g);
}

/**
 * @brief Set the rhs-value of the cell with given position.
 * @param position the position of of the cell
 * @param new_rhs one step lookahead values based on the g-values
 * @return none
 */
void Map::UpdateCellRhs(const std::pair<int, int> &position,
                        const double &new_rhs) {
    grid.at(position.first).at(position.second).UpdateRhs(new_rhs);
}

/**
 * @brief Set the status of the cell with given position.
 * @param position the position of of the cell
 * @param new_status a mark that represent the new status of the cell
 * @return none
 */
void Map::UpdateCellStatus(const std::pair<int, int> &position,
                           const std::string &new_status) {
    grid.at(position.first).at(position.second).UpdateStatus(new_status);
}

/**
 * @brief Set the g-value to infinity of the cell with given position.
 * @param position the position of of the cell
 * @return none
 */
void Map::SetInfiityCellG(const std::pair<int, int> &position) {
    UpdateCellG(position, infinity_cost);
}

/**
 * @brief Compust the cost of from current node to next node.
 * @param current_position current position of of the cell
 * @param next_position next position of of the cell
 * @return cost to travel
 */
double Map::ComputeCost(const std::pair<int, int> &current_position,
                        const std::pair<int, int> &next_position) {
    if (!Availability(next_position)) return infinity_cost;
    if (std::abs(current_position.first - next_position.first) +
        std::abs(current_position.second - next_position.second) == 1)
        return transitional_cost;
    if (std::abs(current_position.first - next_position.first) +
        std::abs(current_position.second - next_position.second) == 2)
        return diagonal_cost;
    else
        return infinity_cost;
}

/**
 * @brief Find eight neighbos that are reachable: not a obstacle nor outside.
 * @param position current position of of the cell
 * @return a set of eight neighbors that are reachable
 */
std::vector<std::pair<int, int>> Map::FindNeighbors(
    const std::pair<int, int> & position) {
    std::vector<std::pair<int, int>> neighbors = {};
    std::vector<int> search_neighbor = {-1, 0, 1};
    for (auto const &i : search_neighbor) {
        for (auto const &j : search_neighbor) {
            auto neighbor = std::make_pair(position.first + i,
                                           position.second + j);
            auto cost = ComputeCost(position, neighbor);
            if (cost == transitional_cost || cost == diagonal_cost)
                neighbors.push_back(neighbor);
        }
    }
    return neighbors;
}

/**
 * @brief Check if the node is not a obstacle nor outside.
 * @param position the position of next position
 * @return true if accessible and flase if not 
 */
bool Map::Availability(const std::pair<int, int> & position) {
    if (position.first < 0 || position.first >= map_size.first) return false;
    if (position.second < 0 || position.second >= map_size.second) return false;
    return grid.at(position.first).at(position.second).CurrentStatus()
           != obstacle_mark;
}

/**
 *
 * @brief Visualize all g-values and rhs-values in the map on the terminal.
 *  
 */
void Map::PrintValue() {
    std::vector<std::string> lines(map_size.second, "-------------");
    std::cout << "Value for shortest path:" << std::endl
        << "(g, rhs): " << std::endl<< " -";
    for (auto line : lines) std::cout << line;
    std::cout << std::endl;
    for (auto const &row : grid) {
        std::cout << " | ";
        for (auto const &cell : row) {
            std::cout << "(" << std::setfill(' ') << std::setw(3)
                      << cell.CurrentG() << ", "  << std::setfill(' ')
                      << std::setw(3) << cell.CurrentRhs() << ") | ";
        }
        std::cout << std::endl << " -";
        for (auto line : lines) std::cout << line;
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

/**
 *
 * @brief Visualize the map and the path the robot has traveled on the terminal.
 *  
 */
void Map::PrintResult() {
    std::vector<std::string> lines(map_size.second, "----");
    std::cout << "Result: " << std::endl
        << "start: " << start_mark << " goal: " << goal_mark << " robot: "
        << robot_mark << " obstacle: " << obstacle_mark << " unknown: "
        << unknown_mark << std::endl;
    std::cout << " -";
    for (auto line : lines) std::cout << line;
    std::cout << std::endl;
    for (auto const &row : grid) {
        std::cout << " | ";
        for (auto const &cell : row) {
            std::cout << cell.CurrentStatus() << " | ";
        }
        std::cout << std::endl << " -";
        for (auto line : lines) std::cout << line;
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
