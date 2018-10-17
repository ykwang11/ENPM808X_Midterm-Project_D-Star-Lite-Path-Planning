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
 * @file Map.h
 * @author Yu-Kai Wang
 * @copyright MIT License
 *
 * @brief D* Lite Path Planning
 *
 * This class controls cells in the map and provides all infomation and 
 * all methods that related to the environment.
 * 
 */

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


