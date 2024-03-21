#include "vehicle.h"

Vehicle::Vehicle(point pos, double speed, double acc, double expect_speed, double headingAngle, double wheelAngle){
    this->pos_c = pos;
    this->speed = speed;
    this->acc = acc;
    this->jerk = 0;
    this->expect_speed = expect_speed;
    this->headingAngle = headingAngle;
    this->wheelAngle = wheelAngle;
    std::cout<< "车辆初始化成功" << std::endl;
}


void Vehicle::ToFrenetMap(ROAD_MAP& map){
    //每200m刷新一次地图
    //以道路中线boundary2作为参考线
    
    

}


void Vehicle::saveVehicleState(){
    // 打开文件
    std::ofstream outputFile("../data/vehiclestate.csv", std::ios_base::app);

    // 检查文件是否成功打开
    if (!outputFile.is_open()) {
        std::cerr << "无法打开文件: " << std::endl;
        return;
    }

    // 写入数据到文件
    // outputFile << 'x' << ',' << 'y' << ',' << 'speed' << ',' << 'acc' << ',' << 'expect_speed' << ',' << 'headingAngle' << ',' << 'wheelAngle' << '\n';
    outputFile << this->pos_c.x << ',' << this->pos_c.y << ',' << 
    this->speed << ',' << this->acc << ',' << this->expect_speed << ',' << this->headingAngle << ',' << this->wheelAngle << '\n';
    

    // 关闭文件
    outputFile.close();

    // std::cout << "数据已保存到 ../data/vehiclestate.csv" << std::endl;
}

void GetNearestObstacle(ROAD_MAP& map){
    // 获取最近障碍物

}


void CruiseControl(ROAD_MAP& m){

}

void Vehicle::update(ROAD_MAP& m){
    // 更新车辆位置
    // 参考线生成。首先检查障碍物，没有障碍物按给定速度expect_speed定速巡航,有动态障碍物判断压速比例决定是否换道，
    // 有静态障碍物判断lanechang or nudge
    // 纵向规划分为三个状态：定速巡航，跟驰，换道


    // 更新车辆速度
    // 更新车辆航向角
    // 更新车辆轮子角度
}
