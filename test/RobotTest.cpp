// Copyright [2018] <Yu-Kai Wang>

#include "Robot.h"
#include <gtest/gtest.h>

TEST(CellRobot, testRobot) {
    // test robot's constructor
    Robot robot_test(std::make_pair(0, 0));
    EXPECT_EQ(robot_test.CurrentPosition(), std::make_pair(0, 0));
    // test robot's action
    robot_test.Move(std::make_pair(1, 2));
    EXPECT_EQ(robot_test.CurrentPosition(), std::make_pair(1, 2));
}
