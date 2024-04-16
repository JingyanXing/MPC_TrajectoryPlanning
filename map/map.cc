#include"map.h"



bool ROAD_MAP::check_block(){
    std::cout << "道路可达性检查------>";
    for(int i = 0; i < this->static_obstacle.size() - 1; i++){
        //障碍物间x方向距离在10m以上认为不堵塞
        if(this->static_obstacle[i + 1].pos.x - this->static_obstacle[i].pos.x < 
        this->static_obstacle[i + 1].length / 2 + this->static_obstacle[i].length / 2 + 10){
            return false;
        }
    }
    std::cout << "无阻塞！" << std::endl;
    return true;
}

ROAD_MAP::ROAD_MAP(double length, int static_obstacle_num, int dynamic_obstacle_num){
    this->static_obstacle = std::vector<Obstacle>();
    this->dynamic_obstacle = std::vector<Obstacle>();
    int points_num = (int) (length / point_gap);
    point p(0,0);
    // 初始化双车道三条边界以及参考线
    for(int i = 0; i <= points_num; i++){
        ROAD_MAP::boundary1.emplace_back(p);
        p.y += ROAD_MAP::width / 2;
        ROAD_MAP::middleline1.emplace_back(p);
        p.y += ROAD_MAP::width / 2;
        ROAD_MAP::boundary2.emplace_back(p);
        p.y += ROAD_MAP::width / 2;
        ROAD_MAP::middleline2.emplace_back(p);
        p.y += ROAD_MAP::width / 2;
        ROAD_MAP::boundary3.emplace_back(p);
        p.x += point_gap;
        p.y = 0;
    }
    std::cout << "地图初始化成功" << std::endl;

    // 随机生成静态障碍物

}

ROAD_MAP::ROAD_MAP(double length, std::vector<Obstacle> static_obstacle, std::vector<Obstacle> dynamic_obstacle){
    int points_num = (int) (length / point_gap);
    point p(0,0);
    this->static_obstacle = static_obstacle;
    this->dynamic_obstacle = dynamic_obstacle;
    // 初始化双车道三条边界以及参考线
    for(int i = 0; i <= points_num; i++){
        ROAD_MAP::boundary1.emplace_back(p);
        p.y += ROAD_MAP::width / 2;
        ROAD_MAP::middleline1.emplace_back(p);
        p.y += ROAD_MAP::width / 2;
        ROAD_MAP::boundary2.emplace_back(p);
        p.y += ROAD_MAP::width / 2;
        ROAD_MAP::middleline2.emplace_back(p);
        p.y += ROAD_MAP::width / 2;
        ROAD_MAP::boundary3.emplace_back(p);
        p.x += point_gap;
        p.y = 0;
    }
    std::cout << "地图初始化成功" << std::endl;
}

ROAD_MAP::ROAD_MAP(){};

ROAD_MAP::~ROAD_MAP(){
    std::cout << "地图析构成功" << std::endl;
}


void ROAD_MAP::updateDynamicObstacle(){
    if(this->dynamic_obstacle.empty()) return;
    for(auto& obs : this->dynamic_obstacle){
        obs.pos.x += obs.speed * 0.1;
        obs.front_left.x += obs.speed * 0.1;
        obs.front_right.x += obs.speed * 0.1;
        obs.rear_left.x += obs.speed * 0.1;
        obs.rear_right.x += obs.speed * 0.1;
    }
}





