// Copyright [2018] <Yu-Kai Wang>

#include "Cell.h"
#include <gtest/gtest.h>

TEST(CellTest, testCell) {
  Cell cell_test;
  EXPECT_EQ(cell_test.CurrentG(), 10000);
  EXPECT_EQ(cell_test.CurrentRhs(), 10000);
  cell_test.UpdateG(10);
  cell_test.UpdateRhs(10);
  EXPECT_EQ(cell_test.CurrentG(), 10);
  EXPECT_EQ(cell_test.CurrentRhs(), 10);
}
