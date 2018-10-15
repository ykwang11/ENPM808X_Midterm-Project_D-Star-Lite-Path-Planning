// Copyright [2018] <Yu-Kai Wang>
// Cell.h
#ifndef INCLUDE_CELL_H_
#define INCLUDE_CELL_H_

#include <string>

class Cell{
 public:
    explicit Cell(const double &);
    double CurrentG() const;
    double CurrentRhs() const;
    std::string CurrentStatus() const;
    void UpdateG(const double &);
    void UpdateRhs(const double &);
    void UpdateStatus(const std::string &);

 private:
    double g = 0;
    double rhs = 0;
    std::string status = "";
};

#endif  // INCLUDE_CELL_H_
