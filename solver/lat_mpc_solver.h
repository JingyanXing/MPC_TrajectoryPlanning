#ifndef LAT_MPC_SOLVER_H
#define LAT_MPC_SOLVER_H
#include <vector>
#include <casadi/casadi.hpp>
#include <algorithm>
#include <iostream>

casadi::DM LatMpcSolver(double curr_s, double curr_l, double curr_heading_angle,
                        double curr_heading_angle_rate, double ref_s,
                        double ref_l, double ref_heading_angle,
                        double ref_heading_angle_rate,
                        std::vector<double> ref_speed,
                        std::vector<double> lower_l,
                        std::vector<double> upper_l, double MAX_WHEELANGLE,
                        double MAX_HEADINGANGLE, double MAX_SPEED,
                        double wheelbase);

casadi::DM LatMpcSolverWheelAngle(
    double curr_l, double curr_l_rate, double curr_heading_angle,
    double curr_heading_angle_rate, double wheelangle,
    std::vector<std::vector<double>> refer_point, std::vector<double> ref_speed,
    std::vector<double> lower_l, std::vector<double> upper_l,
    double MAX_WHEELANGLE, double MAX_WHEELANGLE_RATE, double MAX_HEADINGANGLE,
    double MAX_SPEED, double wheelbase, double vehicle_weigth,
    double front_wheel_corner_stiffness, double rear_wheel_corner_stiffness,
    double moment_of_inertia);

#endif // LAT_MPC_SOLVER_H