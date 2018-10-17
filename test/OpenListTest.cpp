/**
 * MIT Licence
 * Copyright (c) 2018 Yu-Kai Wang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */

/**
 * @file OpenListTest.cpp
 * @author Yu-Kai Wang
 * @copyright MIT License
 *
 * @brief D* Lite Path Planning
 *
 * Test cases for the "OpenList" class
 * 
 */

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
