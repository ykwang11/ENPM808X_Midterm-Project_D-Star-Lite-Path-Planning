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
 * @file TalkingRobotTest.cpp
 * @author Yu-Kai Wang
 * @copyright MIT License
 *
 * @brief D* Lite Path Planning
 *
 * Test cases for the "TalkingRobot" class using GMock.
 * 
 */
#include "mock_Robot.h"
#include "TalkingRobot.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

TEST(TalkingRobotTest, testTalkingRobot) {
    // test MockRobot
    MockRobot robot_mock;
    EXPECT_CALL(robot_mock, getPosition())
        .Times(1);
 
    // test talking robot's action
    //TalkingRobot talker_test(&robot_mock);
    //std::string output;
    //output.append("Accumulated distance is %d\n" 1);

    //testing::internal::CaptureStdout();
 
    //std::string myoutput = testing::internal::GetCapturedStdout();
    //EXPECT_EQ(myoutput, output);
}
