#include"unittest.h"


int main(){
    Test test;
    // 横纵向解耦单元测试
    // test.latSolverUnitTest();
    // 纵向单元测试
    // test.lonSolverUnitTest();
    point init_pos(0,1.75);
    Obstacle static_obs(0, point(80, 1.75), "s1");
    Obstacle static_obs1(0, point(120, 1.75), "s2");
    std::vector<Obstacle> static_obstacle = {static_obs, static_obs1};
    std::vector<Obstacle> dynamic_obstacle;
    ROAD_MAP road(300, static_obstacle, dynamic_obstacle);
    // if(!road.check_block()){
    //     std::cout << "地图道路阻塞，请重新指定生成！" << std::endl;
    //     return 0;
    // }
    Vehicle vehicle(init_pos, 0, 0, 10, 0, 0, 0, road);
    test.run(vehicle, 250);
    system("python ../visual/VehicleStateVisual.py");
    return 0;
}