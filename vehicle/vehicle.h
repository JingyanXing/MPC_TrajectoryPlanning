#ifndef VEHICLE_H
#define VEHICLE_H

#include "../map/map.h"
#include "../solver/lon_mpc_solver.h"
#include "../solver/lat_mpc_solver.h"
#define PI 3.141592653589793238462643383

class Vehicle{
public:
    point pos_c;//笛卡尔坐标系质心位置
    point pos_f;//Frenet坐标系质心位置
    int map_index = 0; //s方向位置对应的地图点，主要是为了减少不必要的重复搜索
    double speed;
    double expect_speed;
    double acc;
    double jerk;
    double headingAngle;//行驶方向
    double wheelAngle;//前轮转角
    double length = 5;//车长
    double width = 1.9;//车辆宽度
    double wheelBase = 3.8;//轴距
    double sensoryRange = 150;//感知范围, 感知其他车辆质心位置
    double MAX_WHEELANGLE = PI / 4;//最大轮胎转角那个
    double MAX_WHEELANGLE_RATE = PI / 6;//轮胎最大转向速率, 30度
    double MAX_HEADINGANGLE = PI / 2;//最大航向角
    double MAX_SPEED = 27;//车辆最大速度
    double MAX_ACCELERATION = 3;//最大加速度
    double MAX_DECELERATION = 7;//最大减速度
    double MAX_JERK = 3;//最大加加速度

    Vehicle(){};
    Vehicle(point pos, double speed, double acc, double expect_speed, double headingAngle, double wheelAngle);
    void update(ROAD_MAP& map);
    void ToFrenetMap(ROAD_MAP& map);
    void GetNearestObstacle(ROAD_MAP& map);
    void saveVehicleState();
    void CruiseControl();
    ~Vehicle(){};
};



//静止障碍物车辆
class StaticVehicle : public Vehicle{

};


#endif
