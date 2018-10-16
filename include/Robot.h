// Copyright [2018] <Yu-Kai Wang>
#ifndef INCLUDE_ROBOT_H_
#define INCLUDE_ROBOT_H_

#include <utility>

class Robot {
 public:
    explicit Robot(const std::pair<int, int> &);
    std::pair<int, int> CurrentPosition() const;
    void Move(const std::pair<int, int> &);
 private:
    std::pair<int, int> position;
};


#endif  // INCLUDE_ROBOT_H_

