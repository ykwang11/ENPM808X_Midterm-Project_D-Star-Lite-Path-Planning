// Copyright [2018] <Yu-Kai Wang>

#include "OpenList.h"
#include <gtest/gtest.h>
#include <string>


TEST(CellOpenList, testOpenList) {
    OpenList openlist_test;
    auto first_node = std::make_pair(3, 2);
    auto second_node = std::make_pair(4, 5);
    auto third_node = std::make_pair(6, 0);
    openlist_test.Insert(0.7, first_node);
    openlist_test.Insert(1.5, second_node);
    openlist_test.Insert(0.5, third_node);
    
    EXPECT_EQ(openlist_test.Top().second.second, third_node.second);
    EXPECT_EQ(openlist_test.Pop().second.second, third_node.second);
    EXPECT_EQ(openlist_test.Top().second.first, first_node.first);
    openlist_test.UpdateKey(3.3 , first_node);
    EXPECT_EQ(openlist_test.Top().second.second, second_node.second);
    EXPECT_FALSE(openlist_test.Find(third_node));
    EXPECT_TRUE(openlist_test.Find(first_node));
}
