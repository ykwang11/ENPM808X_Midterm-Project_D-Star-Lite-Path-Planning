/**
 * @file CellTest.cpp
 * @author Yu-Kai Wang
 * @copyright MIT License
 *
 * @brief D* Lite Path Planning
 *
 * @section Description
 *
 * Test cases for the "Cell" class
 * 
 */

#include "Cell.h"
#include <gtest/gtest.h>
#include <string>

TEST(CellTest, testCell) {
    Cell cell_test(10000.0);
    EXPECT_EQ(cell_test.CurrentG(), 10000.0);
    EXPECT_EQ(cell_test.CurrentRhs(), 10000.0);
    std::string string_test = " ";
    EXPECT_EQ(cell_test.CurrentStatus(), string_test);

    cell_test.UpdateG(10.0);
    cell_test.UpdateRhs(10.0);
    cell_test.UpdateStatus("*");

    EXPECT_EQ(cell_test.CurrentG(), 10.0);
    EXPECT_EQ(cell_test.CurrentRhs(), 10.0);
    std::string updated_string_test = "*";
    EXPECT_EQ(cell_test.CurrentStatus(), updated_string_test);
}
