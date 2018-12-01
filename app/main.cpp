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
* @file Main.cpp
* @author Yu-Kai Wang
* @copyright MIT License
*
* @brief D* Lite Path Planning
*
* This program plans the path of the robot, re-plan new path when the map
* changes and moves the robot to avoid obstacles. That is, it performs the
* D* Lite algorithm.
*
*/

#include <iostream>
#include <vector>
#include <utility>
#include <algorithm>

#include "TalkingRobot.h"
#include "PathPlanner.h"


int main() {
	// Declaration
	auto goal = std::make_pair(0, 0);
	auto start = std::make_pair(2, 4);

	// Setting the environment: obstacles. hidden obstacles, the goal, the robot
	std::vector<std::pair<int, int>> obstacle, hidden;
	obstacle.push_back(std::make_pair(1, 1));
	obstacle.push_back(std::make_pair(0, 2));
	obstacle.push_back(std::make_pair(1, 2));
	hidden.push_back(std::make_pair(2, 2));

	// Initialize
	PathPlanner planner(start, goal, obstacle, hidden);
	TalkingRobot robot(start);

	// Compute shortest path in the beginning
	planner.computeShortestPath(robot.getPosition());
	
	// Keep moving until reach the goal
	while (robot.getPosition() != goal) {
		auto next = planner.getNextPotision(robot.getPosition());
		robot.move(next);
		planner.setMapTrace(robot.getPosition());

		// Print out every step in the journey
		planner.print();

		// Detect environmental change
		auto graphChanged =
			planner.detectHidden(robot.getPosition());

		// Only re-plan path when robot detects change in the environment.
		if (graphChanged) planner.computeShortestPath(robot.getPosition());
	}
	
	std::cout << "Achieved!";
	std::cin.get();
	return 0;
}

