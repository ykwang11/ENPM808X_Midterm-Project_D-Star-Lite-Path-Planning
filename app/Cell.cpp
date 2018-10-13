// Copyright [2018] <Yu-Kai Wang>

#include <Cell.h>

// Constructor
Cell::Cell() {
}

// Deconstructor
Cell::~Cell() {
}

// get function
double Cell::CurrentG() const { return g; }
double Cell::CurrentRhs() const { return rhs; }

// set function
void Cell::UpdateG(double new_g) { g = new_g; }
void Cell::UpdateRhs(double new_rhs) { rhs = new_rhs; }

