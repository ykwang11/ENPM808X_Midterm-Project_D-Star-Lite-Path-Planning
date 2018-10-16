/**
 * @file Robot.cpp
 * @author Yu-Kai Wang
 * @copyright MIT License
 *
 * @brief D* Lite Path Planning
 *
 * @section Description
 *
 * This class saves the information of the robot.
 * 
 */

#include "Robot.h"

/**
 * @brief Constructor
 * @param the start point of the robot
 * @return none
 */
Robot::Robot(const std::pair<int, int> &start_point) { position = start_point; }

/**
 * @brief Get current position.
 * @param none
 * @return the position of the robot
 */
std::pair<int, int> Robot::CurrentPosition() const { return position; }

/**
 * @brief Move the robot.
 * @param next position
 * @return none
 */
void Robot::Move(const std::pair<int, int> &next_position) {
    position = next_position;
}
