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
 * @file Cell.h
 * @author Yu-Kai Wang
 * @copyright MIT License
 *
 * @brief D* Lite Path Planning
 *
 * This class holds the status of each node in the map.
 * g-values: estamated distances to the goal.
 * rhs-values: one step lookahead values based on the g-values.
 *
 */

#ifndef INCLUDE_CELL_H_
#define INCLUDE_CELL_H_

#include <string>

class Cell{
 public:
    explicit Cell(const double &);
    double CurrentG() const;
    double CurrentRhs() const;
    std::string CurrentStatus() const;
    void UpdateG(const double &);
    void UpdateRhs(const double &);
    void UpdateStatus(const std::string &);

 private:
    double g = 0;
    double rhs = 0;
    std::string status = "";
};

#endif  // INCLUDE_CELL_H_
