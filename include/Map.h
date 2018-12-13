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
* This class controls positions' status in the map and provides all infomation and
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

class Map {
 public:
    // different costs
    const double infinity_cost = 100.0;
    const double open_cost = 0.0;
    const double diagonal_cost = 1.4;
    const double transitional_cost = 1.0;

    // different status marks
    const std::string trace_mark = ".";
    const std::string goal_mark = "g";
    const std::string start_mark = "s";
    const std::string obstacle_mark = "x";
    const std::string hidden_mark = "?";

    // constructor and environment initializing
    explicit Map(const int &, const int &);
    void setObstacle(const std::vector<std::pair<int, int>> &,
    const std::vector<std::pair<int, int>> &);
    void setTrace(const std::pair<int, int> &);
    void setGoal(const std::pair<int, int> &);
    void setStart(const std::pair<int, int> &);

    // get method
    std::pair<int, int> getGoal() const;
    double getG(const std::pair<int, int> &) const;
    double getRhs(const std::pair<int, int> &) const;
    double calculateKey(const std::pair<int, int> &) const;
    double getStatus(const std::pair<int, int> &) const;

    // set method
    void setG(const std::pair<int, int> &, const double &);
    void setRhs(const std::pair<int, int> &, const double &);
    void setStatus(const std::pair<int, int> &, const double &);
    void setInfiniteG(const std::pair<int, int> &);

    double computeCost(const std::pair<int, int> &,
    const std::pair<int, int> &);

    std::vector<std::pair<int, int>> findNeighbors(const std::pair<int, int> &);
    bool getClearance(const std::pair<int, int> &);

    // print method
    void printG();
    void printRhs();
    void printResult();

 private:
    std::pair<int, int> size;
    std::vector<std::vector<double>> map;
    std::vector<std::vector<double>> gValue;
    std::vector<std::vector<double>> rhsValue;
    std::pair<int, int> goal;
    std::pair<int, int> start;
};


#endif // INCLUDE_MAP_H_
