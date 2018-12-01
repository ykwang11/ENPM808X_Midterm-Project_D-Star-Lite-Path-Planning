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
* @file Map.cpp
* @author Yu-Kai Wang
* @copyright MIT License
*
* @brief D* Lite Path Planning
*
* This class is the map and provides all infomation and
* all methods that related to the environment.
*
*/

#include "Map.h"
#include <algorithm>

/**
* @brief Constructor.
* @param height the size of the map
* @param width the size of the map
* @return void
*/
Map::Map(const int &height, const int &width) {

	std::vector<std::vector<double>> value(
		height,
		std::vector<double>(width, infinity_cost));
	rhsValue = value;
	gValue = value;
	size = std::make_pair(height, width);

	std::vector<std::vector<bool>> position(
		height,
		std::vector<bool>(width, false));
	obstacle = position;
	hidden = position;
	trace = position;
}

/**
* @brief Add obstacles and change positions' status
* @param obstacle a set of obstackes's position
* @param hidden_obstacle a set of hidden obstackes's position
* @return void
*/
void Map::setObstacle(const std::vector<std::pair<int, int>> &obstacle_,
	const std::vector<std::pair<int, int>> &hidden_) {

	for (auto const &node : hidden_)
		hidden.at(node.first).at(node.second) = true;
	for (auto const &node : obstacle_) {
		obstacle.at(node.first).at(node.second) = true;
		hidden.at(node.first).at(node.second) = false;
	}
}

/**
* @brief Set the trace and change positions' status
* @param goal_ the position of the goal
* @return void
*/
void Map::setTrace(const std::pair<int, int> &position) {
	trace.at(position.first).at(position.second) = true;
}

/**
* @brief Set the goal and change positions' status
* @param goal_ the position of the goal
* @return void
*/
void Map::setGoal(const std::pair<int, int> &goal_) {
	goal = goal_;
}

/**
* @brief Set the start and change positions' status
* @param goal_ the position of the goal
* @return void
*/
void Map::setStart(const std::pair<int, int> &start_) {
	start = start_;
}

/**
* @brief Get the goal's position.
* @return the position of the goal
*/
std::pair<int, int> Map::getGoal() const { return goal; }

/**
* @brief Get the g-value with given position
* @param position the position of interest
* @return g-value
*/
double Map::getG(const std::pair<int, int> &position) const {
	return gValue.at(position.first).at(position.second);
}

/**
* @brief Get the rhs-value with given position
* @param position the position of interest
* @return rhs-value
*/
double Map::getRhs(const std::pair<int, int> &position) const {
	return rhsValue.at(position.first).at(position.second);
}

/**
* @brief Caculat the key (priority in search) with given position
* @param position the position of interest
* @return the key value, which is the priority in next search
*/
double Map::calculateKey(const std::pair<int, int> &position) const {
	return std::min(getG(position), getRhs(position));
}

/**
* @brief Get the status with given position.
* @param position the position of interest
* @return position's status
*/
int Map::getStatus(const std::pair<int, int> &position) const {
	if (obstacle.at(position.first).at(position.second)) {
		return 1;
	} else if (hidden.at(position.first).at(position.second)) {
		return -1;
	} else {
		return 0;
	}
}

/**
* @brief Set the g-value with given position
* @param position the position of interest
* @param new_g new estamated distance to the goal
* @return void
*/
void Map::setG(const std::pair<int, int> &position,
	const double &gValue_) {
	gValue.at(position.first).at(position.second) = gValue_;
}

/**
* @brief Set the rhs-value with given position
* @param position the position of interest
* @param new_rhs one step lookahead values based on the g-values
* @return void
*/
void Map::setRhs(const std::pair<int, int> &position,
	const double &rhsValue_) {
	rhsValue.at(position.first).at(position.second) = rhsValue_;
}

/**
* @brief Set the status with given position
* @param position the position of interest
* @param status_ an integer that represent the new status
* @return void
*/
void Map::setStatus(const std::pair<int, int> &position,
	const int &status_) {
	if (status_ == 1) {
		obstacle.at(position.first).at(position.second) = true;
		hidden.at(position.first).at(position.second) = false;
	} else if (status_ == -1) {
		obstacle.at(position.first).at(position.second) = false;
		hidden.at(position.first).at(position.second) = true;
	} else {
		obstacle.at(position.first).at(position.second) = false;
		hidden.at(position.first).at(position.second) = false;
	}
}

/**
* @brief Set the g-value to infinity with given position
* @param position the position of interest
* @return void
*/
void Map::setInfiniteG(const std::pair<int, int> &position) {
	setG(position, infinity_cost);
}

/**
* @brief Compust the cost of from current node to next node
* @param current_position current position of interest
* @param next_position next position
* @return cost to travel
*/
double Map::computeCost(const std::pair<int, int> &position,
	const std::pair<int, int> &position_) {
	if (!getClearance(position_)) return 5.56; //return infinity_cost;
	if (std::abs(position.first - position_.first) +
		std::abs(position.second - position_.second) == 1)
		return transitional_cost;
	if (std::abs(position.first - position_.first) +
		std::abs(position.second - position_.second) == 2)
		return diagonal_cost;
	else
		return infinity_cost;
}

/**
* @brief Find eight neighbos that are reachable: not a obstacle nor outside.
* @param position current position of interest
* @return a set of eight neighbors that are reachable
*/
std::vector<std::pair<int, int>> Map::findNeighbors(
	const std::pair<int, int> & position) {
	std::vector<std::pair<int, int>> neighbors = {};
	std::vector<int> search_neighbor = { -1, 0, 1 };
	for (auto const &i : search_neighbor) {
		for (auto const &j : search_neighbor) {
			auto neighbor = std::make_pair(position.first + i,
				position.second + j);
			auto cost = computeCost(position, neighbor);
			//std::cout << "neibor: " << cost << ", " << neighbor.first << ", " << neighbor.second << std::endl;
			if (cost == transitional_cost || cost == diagonal_cost)
				neighbors.push_back(neighbor);
		}
	}
	return neighbors;
}

/**
* @brief Check if the node is not a obstacle nor outside.
* @param position the position of next position
* @return true if accessible and flase if not
*/
bool Map::getClearance(const std::pair<int, int> & position) {
	if (position.first < 0 || position.first >= size.first) return false;
	if (position.second < 0 || position.second >= size.second) return false;
	
	return !obstacle.at(position.first).at(position.second);
}

/**
*
* @brief Visualize all g-values in the map on the terminal.
*
*/
void Map::printG() {
	std::vector<std::string> lines(size.second, "------");
	std::cout << "g-value for shortest path:" << std::endl << " -";
	for (auto line : lines) std::cout << line;
	std::cout << std::endl;
	for (auto gRow : gValue) {
		std::cout << " | ";
		for (auto const g : gRow) {
			std::cout << std::setfill(' ') << std::setw(3) << g << " | ";
		}
		std::cout << std::endl << " -";
		for (auto line : lines) std::cout << line;
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

/**
*
* @brief Visualize all rhs-values in the map on the terminal.
*
*/
void Map::printRhs() {
	std::vector<std::string> lines(size.second, "------");
	std::cout << "rhs-value for shortest path:" << std::endl << " -";
	for (auto line : lines) std::cout << line;
	std::cout << std::endl;
	for (auto rhsRow : rhsValue) {
		std::cout << " | ";
		for (auto const rhs : rhsRow) {
			std::cout << std::setfill(' ') << std::setw(3) << rhs << " | ";
		}
		std::cout << std::endl << " -";
		for (auto line : lines) std::cout << line;
		std::cout << std::endl;
	}
	std::cout << std::endl;
}


/**
*
* @brief Visualize the map and the path the robot has traveled on the terminal.
*
*/
void Map::printResult() {
	std::vector<std::string> lines(size.second, "----");
	std::cout << "Result: " << std::endl
		<< "start: " << start_mark << " goal: " << goal_mark << " robot: "
		<< trace_mark << " obstacle: " << obstacle_mark << " unknown: "
		<< hidden_mark << std::endl;
	std::cout << " -";
	for (auto line : lines) std::cout << line;
	std::cout << std::endl;

	int i = 0;
	while (i < size.first) {
		std::cout << " | ";
		int j = 0;
		while (j < size.second) {
			std::string mark = " ";
			if (obstacle.at(i).at(j)) {
				mark = obstacle_mark;
			} else if (hidden.at(i).at(j)) {
				mark = hidden_mark;
			} else if (trace.at(i).at(j)) {
				mark = trace_mark;
			} else if (goal == std::make_pair(i, j)) {
				mark = goal_mark;
			} else if (start == std::make_pair(i, j)) {
				mark = start_mark;
			}
			std::cout << mark << " | ";
			j++;
		}
		std::cout << std::endl << " -";
		for (auto line : lines) std::cout << line;
		std::cout << std::endl;
		i++;
	}

	std::cout << std::endl;
}
