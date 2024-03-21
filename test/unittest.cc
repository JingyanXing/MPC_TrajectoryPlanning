#include "unittest.h"


int Test::checkRefPoint(int curr_point_index, double vehicle_pos_x, ReferenceLine& refer_line){
    int res = curr_point_index;
    while(vehicle_pos_x > refer_line.refer_line[res].x){
        res++;
    }
    return res;
}

//纵向mpc求解器单元测试
void Test::lonSolverUnitTest(){
    std::cout << "----------------纵向mpc求解器单元测试----------------" << std::endl;
    point init_pos(0,1.75);
    Vehicle vehicle(init_pos, 0, 0, 5, 0, 0);
    std::vector<double> state = {0, 0, 0};
    std::vector<double> target_state = {0, 15, 0};
    std::vector<double> s;
    std::vector<double> v;
    std::vector<double> a;
    for(int i = 0; i < 200; i++){
        s.push_back(state[0]);
        v.push_back(state[1]);
        a.push_back(state[2]);
        casadi::DM j = LonMpcSolver(state[0], state[1], state[2], target_state[0], target_state[1], target_state[2], 
                                            vehicle.MAX_SPEED, vehicle.MAX_ACCELERATION, vehicle.MAX_DECELERATION, vehicle.MAX_JERK)(0);
        state[0] += 0.1 * state[1];
        state[1] += 0.1 * state[2];
        state[2] += (double)j;
        target_state[0] += 0.1 * target_state[1];
    }
    saveToCSV(s, "s.csv");
    saveToCSV(v, "v.csv");
    saveToCSV(a, "a.csv");
    std::cout << "---------------纵向mpc求解器单元测试完成---------------" << std::endl;
    system("python3 ../visual/lonSolverUnitTest.py");
}
//横向mpc求解器单元测试
void Test::latSolverUnitTest(){
    std::cout << "----------------横纵向mpc求解器单元测试----------------" << std::endl;
    //初始化车辆状态
    point init_pos(0,1.75);
    Obstacle static_obs(0, point(40, 1.75), "s1");
    Obstacle static_obs1(0, point(100, 5.25), "s2");
    std::vector<Obstacle> static_obstacle = {static_obs, static_obs1};
    std::vector<Obstacle> dynamic_obstacle;
    ROAD_MAP road(300, static_obstacle, dynamic_obstacle);
    // if(!road.check_block()){
    //     std::cout << "地图道路阻塞，请重新指定生成！" << std::endl;
    //     return 0;
    // }
    Vehicle vehicle(init_pos, 0, 0, 5, 0, 0);
    ReferenceLine refer_line(vehicle.pos_c, vehicle.width, road);

    //检查数据文件是否存在
    std::string filename = "../data/vehiclestate.csv";

    std::ifstream file(filename);
    if (file.good()) {
        file.close(); // 关闭文件流，以便能够删除文件
        if (std::remove(filename.c_str()) != 0) {
            std::cout << "Error deleting file " << filename << std::endl;
        } else {
            std::cout << "File " << filename << " successfully deleted." << std::endl;
        }
    } else {
        std::cout << "File " << filename << " does not exist." << std::endl;
    }
    double heading_angle_rate = 0; // 初始变化率
    for(int step = 0; step < 200; step++){
        //基于当前位置向前查找参考点
        std::vector<std::vector<double>> refer_point;
        for(int i = 1; i < 21; i++){
            int index = this->checkRefPoint(vehicle.map_index, vehicle.pos_c.x + vehicle.expect_speed * i / 10, refer_line);
            refer_point.push_back({refer_line.refer_line[index].x, 
                                refer_line.refer_line[index].y,
                                vehicle.expect_speed,
                                (double)atan2((refer_line.refer_line[index].y - refer_line.refer_line[index - 1].y),
                                        (refer_line.refer_line[index].x - refer_line.refer_line[index - 1].x))});
        }
        //纵向mpc求解器
        casadi::DM j = LonMpcSolver(vehicle.pos_c.x, vehicle.speed, vehicle.acc, refer_point.back()[0], 
                                            refer_point.back()[2], 0, vehicle.MAX_SPEED, vehicle.MAX_ACCELERATION, 
                                            vehicle.MAX_DECELERATION, vehicle.MAX_JERK);
        std::vector<double> ref_speed = {vehicle.speed + vehicle.acc * 0.1};
        double tmp_acc = vehicle.acc;
        for(int x = 0; x < 19; x++){
            tmp_acc += (double)j(x);
            ref_speed.push_back(vehicle.speed + tmp_acc * 0.1);
        }
        std::vector<double> lower_l(20, 0);
        std::vector<double> upper_l(20, 7);
        //横向mpc求解器
        casadi::DM heading_angle_rate_dot = LatMpcSolver(vehicle.pos_c.x, vehicle.pos_c.y, vehicle.headingAngle, heading_angle_rate, refer_point.back()[0], 
                                            refer_point.back()[1], refer_point.back()[3], refer_point.back()[3] - refer_point[19][3], 
                                            ref_speed, lower_l, upper_l, vehicle.MAX_WHEELANGLE, vehicle.MAX_HEADINGANGLE,vehicle.MAX_SPEED, vehicle.wheelBase)(0);
        //更新车辆状态
        vehicle.pos_c.x += vehicle.speed * 0.1 * cos(vehicle.headingAngle);
        vehicle.pos_c.y += vehicle.speed * 0.1 * sin(vehicle.headingAngle);
        vehicle.headingAngle += (double)heading_angle_rate_dot * 0.1;
        vehicle.wheelAngle = (double)atan2(heading_angle_rate_dot * vehicle.wheelBase, vehicle.speed + 0.01); //避免速度为0
        vehicle.speed += vehicle.acc * 0.1;
        vehicle.acc += (double)j(0);
        vehicle.saveVehicleState();
        int new_map_index = vehicle.map_index;
        while(refer_line.refer_line[new_map_index].x < vehicle.pos_c.x) new_map_index++;
        vehicle.map_index = new_map_index;
    }

    saveLaneToCSV(refer_line.refer_line, "latSolverUnitTest_refer_lane.csv");
    std::cout << "参考线已保存至 ../data/latSolverUnitTest_refer_lane.csv" << std::endl;
    std::cout << "车辆状态已保存至 ../data/vehiclestate.csv" << std::endl;
    std::cout << "--------------横纵向mpc求解器单元测试完成--------------" << std::endl;
    system("python3 ../visual/latSolverUnitTest.py");
}

