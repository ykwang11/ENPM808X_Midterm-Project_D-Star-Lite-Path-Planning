// Copyright [2018] <Yu-Kai Wang>

#include "Cell.h"
#include <gtest/gtest.h>

TEST(CellTest, testCell) {
  Cell cell_test;
  EXPECT_EQ(cell_test.currentG(), 10000);
  EXPECT_EQ(cell_test.currentRhs(), 10000);
  cell_test.updateG(10);
  cell_test.updateRhs(10);
  EXPECT_EQ(cell_test.currentG(), 10);
  EXPECT_EQ(cell_test.currentRhs(), 10);
}
