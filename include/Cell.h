// Cell.h
#pragma once

class Cell
{
public:
	Cell();
	~Cell();
	double CurrentG() const;
	double CurrentRhs() const;
	void UpdateG(double);
	void UpdateRhs(double);

private:
	double g = 10000; // Initialized to infinity
	double rhs = 10000; // Initialized to infinity
};
