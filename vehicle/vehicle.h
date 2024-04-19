#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include "map.h"
#include "lon_mpc_solver.h"
#include "lat_mpc_solver.h"
#define PI 3.141592653589793238462643383

class Vehicle{
public:
    point pos_c;//笛卡尔坐标系质心位置
    point pos_f;//Frenet坐标系质心位置
    int state = 0; //车辆运动状态，0表示跟驰，1表示开始换道，2表示正在换道
    int target_lane = -1; // 目标车道，这一值辅助参考线切换
    int lat_solver_mode = 0; // 横向规划MPC模型切换
    bool is_curise = false; //巡航状态，false表示换道禁用
    double speed;
    double expect_speed;  //期望速度，主要用于纵向求解器求解
    double curise_speed; //巡航速度，给定的期望行驶速度，全程不变
    double acc;
    double jerk;
    double headingAngle;//行驶方向
    double headingAngleRate;//行驶方向变化率
    double wheelAngle;//前轮转角
    double length = 5;//车长
    double width = 1.9;//车辆宽度
    double wheelBase = 2.8;//轴距
    double weight = 1500;//车辆总质量,kg
    double front_wheel_corner_stiffness = 30000;//前轮转向刚度
    double rear_wheel_corner_stiffness = 30000;//后轮转向刚度
    double moment_of_inertia = 2300; //转动惯量
    double sensoryRange = 150;//感知范围, 感知其他车辆质心位置
    double MAX_WHEELANGLE = PI / 4;//最大轮胎转角那个
    double MAX_WHEELANGLE_RATE = PI / 6;//轮胎最大转向速率, 30度
    double MAX_HEADINGANGLE = PI / 2;//最大航向角
    double MAX_SPEED = 27;//车辆最大速度
    double MAX_ACCELERATION = 3;//最大加速度
    double MAX_DECELERATION = 7;//最大减速度
    double MAX_JERK = 3;//最大加加速度
    std::vector<point> refer_line;//参考线
    ROAD_MAP map;
    std::string planning_trajectory = "";
    std::vector<Obstacle> obstacle_in_range; //存储感知范围内所有障碍物
    std::vector<Obstacle> obstacle_in_lane1; //存储感知范围内当前车道障碍物
    std::vector<Obstacle> obstacle_in_lane2; //存储感知范围内目标车道障碍物


    Vehicle(){};
    Vehicle(point pos, double speed, double acc, double curise_speed, double headingAngle, double headingAngleRate, double wheelAngle, ROAD_MAP& map);
    void drive();
    void setCuriseMode();
    void setLatMpcMode(int mode);
    void ToFrenetMap(ROAD_MAP& map);
    void GetObstacleInSensoryRange();
    void saveVehicleState();
    void updateReferenceLine();
    void InsertLaneFollowPoint(double length);
    void InsertLaneChangePoint(point start_point, point end_point);
    int checkLaneID(point pos);
    int checkRefPoint(double vehicle_pos_x);
    double IDMBasedSpeed(double front_vehicle_speed, point front_vehicle_pos);
    ~Vehicle(){};
};



//静止障碍物车辆
class StaticVehicle : public Vehicle{

};


#endif
