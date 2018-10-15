// Copyright [2018] <Yu-Kai Wang>
#include "Cell.h"

// Constructor
Cell::Cell(const double &initial_num) {
    g = initial_num;
    rhs = initial_num;
    status = " ";
}
// get function
double Cell::CurrentG() const { return g; }
double Cell::CurrentRhs() const { return rhs; }
std::string Cell::CurrentStatus() const { return status; }

// set function
void Cell::UpdateG(const double &new_g) { g = new_g; }
void Cell::UpdateRhs(const double &new_rhs) { rhs = new_rhs; }
void Cell::UpdateStatus(const std::string &new_status) { status = new_status; }
