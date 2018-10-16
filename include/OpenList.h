// Copyright [2018] <Yu-Kai Wang>
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
