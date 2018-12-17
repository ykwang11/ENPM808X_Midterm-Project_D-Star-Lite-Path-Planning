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

#ifndef INCLUDE_PATHPLANNER_H_
#define INCLUDE_PATHPLANNER_H_

#include <iostream>
#include "Map.h"
#include "OpenList.h"

class PathPlanner {
 public:
    PathPlanner(std::pair<int, int>, std::pair<int, int>, std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>);

    void computeShortestPath(const std::pair<int, int> &);
    void updateVertex(const std::pair<int, int> &);
    double getMinRhs(const std::pair<int, int> &);
    std::pair<int, int> getNextPotision(const std::pair<int, int> &);
    bool detectHidden(const std::pair<int, int> &);
    void setMapTrace(const std::pair<int, int> &);
    void print();

 private:
    Map map = Map(5, 5);
    OpenList openlist;
};

#endif // INCLUDE_PATHPLANNER_H_
