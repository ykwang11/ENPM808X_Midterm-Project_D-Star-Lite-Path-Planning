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
 * @file OpenList.h
 * @author Yu-Kai Wang
 * @copyright MIT License
 *
 * @brief D* Lite Path Planning
 *
 * @section Description
 *
 * This class saves candidate nodes to search in minimum heap data structure.
 * 
 */


#ifndef INCLUDE_OPENLIST_H_
#define INCLUDE_OPENLIST_H_

#include <iostream>
#include <tuple>
#include <vector>
#include <utility>
#include <algorithm>

class OpenList {
 public:
    void Insert(const double &, const std::pair<int, int> &);
    void UpdateKey(const double &, const std::pair<int, int> &);
    void Remove(const std::pair<int, int> &);
    std::pair<double, std::pair<int, int>> Top() const;
    std::pair<double, std::pair<int, int>> Pop();
    bool Find(const std::pair<int, int> &) const;
 private:
    std::vector<std::tuple<double, int, int>> priority_queue;
};


#endif  // INCLUDE_OPENLIST_H_
