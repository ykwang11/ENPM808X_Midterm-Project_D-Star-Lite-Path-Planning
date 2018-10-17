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
 * @file Cell.cpp
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

#include "Cell.h"

/**
 * @brief Constructor.
 * @param initial_num a number for the inital g-value and rhs-value
 * @return none
 */
Cell::Cell(const double &initial_num) {
    g = initial_num;
    rhs = initial_num;
    status = " ";
}

/**
 * @brief Get current g-value.
 * @return g-value
 */
double Cell::CurrentG() const { return g; }

/**
 * @brief Get current rhs-value.
 * @return rhs-value
 */
double Cell::CurrentRhs() const { return rhs; }

/**
 * @brief Get current status: obstacle, unknown obstacle, goal, or start point.
 * @return mark that represent the status
 */
std::string Cell::CurrentStatus() const { return status; }

/**
 * @brief Set new g-value.
 * @param new_g new estamated distance to the goal
 * @return none
 */
void Cell::UpdateG(const double &new_g) { g = new_g; }

/**
 * @brief Set new rhs-value.
 * @param new_rhs one step lookahead values based on the g-values
 * @return none
 */
void Cell::UpdateRhs(const double &new_rhs) { rhs = new_rhs; }

/**
 * @brief Set new status.
 * @param new_status a mark represent new status
 * @return none
 */
void Cell::UpdateStatus(const std::string &new_status) { status = new_status; }
