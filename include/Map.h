// Copyright [2018] <Yu-Kai Wang>
#ifndef INCLUDE_MAP_H_
#define INCLUDE_MAP_H_

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <utility>
#include "Cell.h"

class Map {
 public:
    // different costs
    const double infinity_cost = 100.0;
    const double diagonal_cost = 2.5;
    const double transitional_cost = 1.0;

    // different status marks
    const std::string robot_mark = ".";
    const std::string goal_mark = "g";
    const std::string start_mark = "s";
    const std::string obstacle_mark = "x";
    const std::string unknown_mark = "?";

    // constructor and environment initializing
    explicit Map(const int &, const int &);
    void AddObstacle(const std::vector<std::pair<int, int>> &,
                     const std::vector<std::pair<int, int>> &);
    void SetGoal(const std::pair<int, int> &);

    // get method
    std::pair<int, int> GetGoal() const;
    double CurrentCellG(const std::pair<int, int> &) const;
    double CurrentCellRhs(const std::pair<int, int> &) const;
    double CalculateCellKey(const std::pair<int, int> &) const;
    std::string CurrentCellStatus(const std::pair<int, int> &) const;

    // set method
    void UpdateCellG(const std::pair<int, int> &, const double &);
    void UpdateCellRhs(const std::pair<int, int> &, const double &);
    void UpdateCellStatus(const std::pair<int, int> &, const std::string &);
    void SetInfiityCellG(const std::pair<int, int> &);

    double ComputeCost(const std::pair<int, int> &,
                       const std::pair<int, int> &);

    std::vector<std::pair<int, int>> FindNeighbors(const std::pair<int, int> &);
    bool Availability(const std::pair<int, int> &);

    // print method
    void PrintValue();
    void PrintResult();

 private:
    std::pair<int, int> map_size;
    std::vector<std::vector<Cell>> grid;
    std::pair<int, int> goal;
};


#endif  // INCLUDE_MAP_H_


