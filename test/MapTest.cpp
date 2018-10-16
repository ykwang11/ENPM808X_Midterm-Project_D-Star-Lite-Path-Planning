/**
 * @file MapTest.cpp
 * @author Yu-Kai Wang
 * @copyright MIT License
 *
 * @brief D* Lite Path Planning
 *
 * @section Description
 *
 * Test cases for the "Map" class
 * 
 */

#include "Map.h"
#include "Cell.h"
#include <gtest/gtest.h>

TEST(MapTest, testMapGetMethod) {
    // declare a map
    Map map_test(5, 5);

    // add obstacles
    std::vector<std::pair<int, int>> obstacles_for_test = {};
    std::vector<std::pair<int, int>> unknown_for_test = {};
    obstacles_for_test.push_back(std::make_pair(1, 3));
    unknown_for_test.push_back(std::make_pair(3, 2));
    map_test.AddObstacle(obstacles_for_test, unknown_for_test);

    // add goal
    auto goal_for_test = std::make_pair(4, 2);
    map_test.SetGoal(goal_for_test);

    // test map
    auto node_for_test = std::make_pair(1, 1);
    std::string status_for_test = " ";
    EXPECT_EQ(map_test.GetGoal(), goal_for_test);
    EXPECT_EQ(map_test.CurrentCellG(node_for_test), 100.0);
    EXPECT_EQ(map_test.CurrentCellRhs(node_for_test), 100.0);
    EXPECT_EQ(map_test.CalculateCellKey(node_for_test), 100.0);
    EXPECT_EQ(map_test.CurrentCellStatus(node_for_test), status_for_test);
}


TEST(MapTest, testMapSetMethod) {
    // declare a map
    Map map_test(8, 9);

    // add obstacles
    std::vector<std::pair<int, int>> obstacles_for_test = {};
    std::vector<std::pair<int, int>> unknown_for_test = {};
    obstacles_for_test.push_back(std::make_pair(3, 3));
    unknown_for_test.push_back(std::make_pair(3, 2));
    map_test.AddObstacle(obstacles_for_test, unknown_for_test);

    // add goal
    auto goal_for_test = std::make_pair(4, 2);
    map_test.SetGoal(goal_for_test);

    // set map
    auto node_for_test = std::make_pair(5, 5);
    auto node2_for_test = std::make_pair(3, 5);
    std::string status_for_test = "This is unit Testing";
    map_test.UpdateCellG(node_for_test, 5566.0);
    map_test.SetInfiityCellG(node2_for_test);
    map_test.UpdateCellRhs(node_for_test, 7878.0);
    map_test.UpdateCellStatus(node_for_test, status_for_test);

    // test map
    EXPECT_EQ(map_test.CurrentCellG(node_for_test), 5566.0);
    EXPECT_EQ(map_test.CurrentCellG(node2_for_test), 100.0);
    EXPECT_EQ(map_test.CurrentCellRhs(node_for_test), 7878.0);
    EXPECT_EQ(map_test.CurrentCellStatus(node_for_test), status_for_test);
}


TEST(MapTest, testMapOtherMethod) {
    // declare a map
    Map map_test(100, 70);

    // add obstacles
    std::vector<std::pair<int, int>> obstacles_for_test = {};
    std::vector<std::pair<int, int>> unknown_for_test = {};
    obstacles_for_test.push_back(std::make_pair(99, 3));
    unknown_for_test.push_back(std::make_pair(3, 22));
    map_test.AddObstacle(obstacles_for_test, unknown_for_test);

    // test map
    auto node_for_test = std::make_pair(99, 2);
    unsigned int size = 4;
    EXPECT_EQ(map_test.FindNeighbors(node_for_test).size(), size);
    EXPECT_FALSE(map_test.Availability(std::make_pair(99, 3)));
    EXPECT_EQ(map_test.ComputeCost(node_for_test,
                                   std::make_pair(99, 3)), 100.0);
}

TEST(MapTest, testMapPrintValue) {
    // declare a map
    Map map_test(2, 2);

    // test map
    std::string output;
    output.append("Value for shortest path:\n");
    output.append("(g, rhs): \n");
    output.append(" ---------------------------\n");
    output.append(" | (100, 100) | (100, 100) | \n");
    output.append(" ---------------------------\n");
    output.append(" | (100, 100) | (100, 100) | \n");
    output.append(" ---------------------------\n\n");

    testing::internal::CaptureStdout();
    map_test.PrintValue();
    std::string myoutput = testing::internal::GetCapturedStdout();
    EXPECT_EQ(myoutput, output);
}


TEST(MapTest, testMapPrintResult) {
    // declare a map
    Map map_test(2, 2);

    // add obstacles
    std::vector<std::pair<int, int>> obstacles_for_test = {};
    std::vector<std::pair<int, int>> unknown_for_test = {};
    obstacles_for_test.push_back(std::make_pair(0, 1));
    unknown_for_test.push_back(std::make_pair(1, 0));
    map_test.AddObstacle(obstacles_for_test, unknown_for_test);

    // add goal
    auto goal_for_test = std::make_pair(1, 1);
    map_test.SetGoal(goal_for_test);

    // test map
    std::string output;
    output.append("Result: \n");
    output.append("start: s goal: g robot: . obstacle: x unknown: ?\n");
    output.append(" ---------\n");
    output.append(" |   | x | \n");
    output.append(" ---------\n");
    output.append(" | ? | g | \n");
    output.append(" ---------\n\n");

    testing::internal::CaptureStdout();
    map_test.PrintResult();
    std::string myoutput = testing::internal::GetCapturedStdout();
    EXPECT_EQ(myoutput, output);
}
