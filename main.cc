#include"unittest.h"


int main(){
    Test test;
    // 横纵向解耦单元测试
    // test.latSolverUnitTest();
    // 纵向单元测试
    // test.lonSolverUnitTest();
    point init_pos(0, 1.75);
    Obstacle dynamic_obs1(1, point(60, 5.25), "d1", 5);
    Obstacle dynamic_obs2(1, point(120, 5.25), "d2", 10);
    Obstacle dynamic_obs3(1, point(200, 1.75), "d3", 10);
    Obstacle static_obs1(0, point(80, 1.75), "s1");
    std::vector<Obstacle> static_obstacle = {static_obs1};
    std::vector<Obstacle> dynamic_obstacle = {dynamic_obs1,dynamic_obs2, dynamic_obs3};
    ROAD_MAP road(1000, static_obstacle, dynamic_obstacle);
    Vehicle vehicle(init_pos, 0, 0, 10, 0, 0, 0., road);
    vehicle.setLatMpcMode(1);
    test.run(vehicle, 300);
    system("python ../visual/VehicleStateVisual.py");
    return 0;
}