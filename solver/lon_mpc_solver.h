#ifndef LON_MPC_SOLVER_H
#define LON_MPC_SOLVER_H
#include <vector>
#include <casadi/casadi.hpp>
#include <algorithm>
#include <iostream>

casadi::DM LonMpcSolver(double curr_s, double curr_v, double curr_a,
                        double ref_s, double ref_v, double ref_a,
                        double max_speed, double max_acc, double max_decel,
                        double max_jerk);

#endif //LON_MPC_SOLVER_H