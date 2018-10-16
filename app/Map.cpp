// Copyright [2018] <Yu-Kai Wang>
#include "Map.h"
#include <algorithm>

// Constructor
Map::Map(const int &height, const int &width) {
    std::vector<std::vector<Cell>> new_grid(
        height,
        std::vector<Cell>(width, Cell(infinity_cost)));
    grid = new_grid;
    map_size = std::make_pair(height, width);
}

void Map::AddObstacle(const std::vector<std::pair<int, int>> &obstacle,
                      const std::vector<std::pair<int, int>> &hidden_obstacle) {
    for (auto const &node : obstacle)
        grid.at(node.first).at(node.second).UpdateStatus(obstacle_mark);
    for (auto const &node : hidden_obstacle)
        grid.at(node.first).at(node.second).UpdateStatus(unknown_mark);
}

void Map::SetGoal(const std::pair<int, int> &new_goal) {
    goal = new_goal;
    UpdateCellStatus(new_goal, goal_mark);
}

std::pair<int, int> Map::GetGoal() const { return goal; }

double Map::CurrentCellG(const std::pair<int, int> &position) const {
    return grid.at(position.first).at(position.second).CurrentG();
}

double Map::CurrentCellRhs(const std::pair<int, int> &position) const {
    return grid.at(position.first).at(position.second).CurrentRhs();
}

double Map::CalculateCellKey(const std::pair<int, int> &position) const {
    return std::min(CurrentCellG(position), CurrentCellRhs(position));
}

std::string Map::CurrentCellStatus(const std::pair<int, int> &position) const {
    return grid.at(position.first).at(position.second).CurrentStatus();
}

void Map::UpdateCellG(const std::pair<int, int> &position,
                      const double &new_g) {
    grid.at(position.first).at(position.second).UpdateG(new_g);
}

void Map::UpdateCellRhs(const std::pair<int, int> &position,
                        const double &new_rhs) {
    grid.at(position.first).at(position.second).UpdateRhs(new_rhs);
}

void Map::UpdateCellStatus(const std::pair<int, int> &position,
                           const std::string &new_status) {
    grid.at(position.first).at(position.second).UpdateStatus(new_status);
}

void Map::SetInfiityCellG(const std::pair<int, int> &position) {
    UpdateCellG(position, infinity_cost);
}

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

bool Map::Availability(const std::pair<int, int> & position) {
    if (position.first < 0 || position.first >= map_size.first) return false;
    if (position.second < 0 || position.second >= map_size.second) return false;
    return grid.at(position.first).at(position.second).CurrentStatus()
           != obstacle_mark;
}

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
