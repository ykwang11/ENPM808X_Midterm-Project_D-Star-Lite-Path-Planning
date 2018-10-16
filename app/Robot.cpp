// Copyright [2018] <Yu-Kai Wang>

#include "Robot.h"

Robot::Robot(const std::pair<int, int> &start_point) { position = start_point; }

std::pair<int, int> Robot::CurrentPosition() const { return position; }

void Robot::Move(const std::pair<int, int> &next_position) {
    position = next_position;
}
