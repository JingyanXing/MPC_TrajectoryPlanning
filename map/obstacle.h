#ifndef OBSTACLE_H
#define OBSTACLE_H
#include<vector>
#include<iostream>
#include<math.h>
#include "point.h"

class Obstacle{
public:
    //障碍物属性
    double length;
    double width;
    bool is_dynamic = false;
    std::string name;
    double speed = 0.0;
    point pos;
    //生成轮廓位置
    //              头  
    // front_left-------front_right
    //           |     |
    //           |     |
    // rear_left -------rear_right
    //              尾  
    point front_right;
    point front_left;
    point rear_right;
    point rear_left;
    //障碍物类型type
    //静态障碍物初始化
    Obstacle(int type, point pos, std::string name);
    Obstacle(int type, point pos, std::string name, double speed);
    ~Obstacle() = default;
};

#endif


