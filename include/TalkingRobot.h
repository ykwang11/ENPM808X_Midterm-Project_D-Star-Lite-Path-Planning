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
* @file TalkingRobot.h
* @author Yu-Kai Wang
* @copyright MIT License
*
* @brief D* Lite Path Planning
*
* The talking robot talks how long it has walked everytime it moves.
* This is a dummy inherited class for GMock practice.
*/

#ifndef INCLUDE_TALKINGROBOT_H_
#define INCLUDE_TALKINGROBOT_H_
#include "Robot.h"
#include <iostream>

class TalkingRobot : public Robot
{
public:
	TalkingRobot(const std::pair<int, int> &);
	void move(const std::pair<int, int> &);
private:
	int distance;
};

#endif  // INCLUDE_TALKINGROBOT_H_

