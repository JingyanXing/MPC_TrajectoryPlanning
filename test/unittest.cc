#include "unittest.h"


//纵向mpc求解器单元测试
void Test::lonSolverUnitTest(){
    std::cout << "----------------纵向mpc求解器单元测试----------------" << std::endl;
    point init_pos(0,1.75);
    Obstacle static_obs(0, point(40, 1.75), "s1");
    Obstacle static_obs1(0, point(100, 5.25), "s2");
    std::vector<Obstacle> static_obstacle = {static_obs, static_obs1};
    std::vector<Obstacle> dynamic_obstacle;
    ROAD_MAP road(300, static_obstacle, dynamic_obstacle);
    Vehicle vehicle(init_pos, 0, 0, 5, 0, 0, 0, road);
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
    // Obstacle static_obs(0, point(40, 1.75), "s1");
    Obstacle static_obs1(0, point(80, 1.75), "s2");
    std::vector<Obstacle> static_obstacle = {static_obs1};
    std::vector<Obstacle> dynamic_obstacle;
    ROAD_MAP road(300, static_obstacle, dynamic_obstacle);
    // if(!road.check_block()){
    //     std::cout << "地图道路阻塞，请重新指定生成！" << std::endl;
    //     return 0;
    // }
    Vehicle vehicle(init_pos, 0, 0, 10, 0, 0, 0, road);
    // ReferenceLine refer_line(vehicle.pos_c, vehicle.width, road);

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

    
    for(int step = 0; step < 200; step++){
        vehicle.drive();
    }

    saveLaneToCSV(vehicle.refer_line, "latSolverUnitTest_refer_lane.csv");
    std::cout << "参考线已保存至 ../data/latSolverUnitTest_refer_lane.csv" << std::endl;
    std::cout << "车辆状态已保存至 ../data/vehiclestate.csv" << std::endl;
    std::cout << "--------------横纵向mpc求解器单元测试完成--------------" << std::endl;
    system("python3 ../visual/VehicleStateVisual.py");
}


void Test::run(Vehicle& vehicle, int step){
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

    // 1. 创建客户端，并连接到服务端
    //AF_INET 参数指定了IPv4地址族，SOCK_STREAM 参数指定了TCP协议。socket() 函数返回的整数是套接字的文件描述符。
    int sock_client = socket(AF_INET, SOCK_STREAM, 0); //#include <arpa/inet.h>
    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); //本地主机地址
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8888); //端口号
    connect(sock_client, (sockaddr*)&server_addr, sizeof(sockaddr));

    for(int i = 0; i < step; i++){
        // 2. 发送数据，并接受服务端数据
        vehicle.drive();
        // 将车辆状态发送给服务端
        // x,y,speed,headingangle,wheelangle
        std::string vehicle_info =  (std::to_string(vehicle.pos_c.x) + ',' 
                                    + std::to_string(vehicle.pos_c.y) + ','
                                    + std::to_string(vehicle.speed) + ','
                                    + std::to_string(vehicle.headingAngle) + ','
                                    + std::to_string(vehicle.wheelAngle) + ','
                                    + vehicle.planning_trajectory) + '\n';

        std::string obstacle_info = "";
        // 每个障碍物的四个顶点坐标
        for(auto& obs : vehicle.obstacle_in_range){
            obstacle_info += std::to_string(obs.rear_right.x) + ',' + std::to_string(obs.rear_right.y) + ',';
            obstacle_info += std::to_string(obs.front_right.x) + ',' + std::to_string(obs.front_right.y) + ',';
            obstacle_info += std::to_string(obs.front_left.x) + ',' + std::to_string(obs.front_left.y) + ',';
            obstacle_info += std::to_string(obs.rear_left.x) + ',' + std::to_string(obs.rear_left.y) + '\n';
        }

        std::string refer_line_info = "";
        for(int i = 0; i < vehicle.refer_line.size(); i++){
            if(i % 10 == 0) 
                refer_line_info += std::to_string(vehicle.refer_line[i].x) + ',' + std::to_string(vehicle.refer_line[i].y) + ',';
        }
        refer_line_info.pop_back();
        refer_line_info += '\n'; 
        refer_line_info += '*'; //消息结束识别符

        const char* send_info1 = vehicle_info.c_str();
        const char* send_info2 = obstacle_info.c_str();
        const char* send_info3 = refer_line_info.c_str();
        send(sock_client, send_info1, strlen(send_info1), 0);
        send(sock_client, send_info2, strlen(send_info2), 0);
        send(sock_client, send_info3, strlen(send_info3), 0);

        char recv_info[50];
        recv(sock_client, recv_info, sizeof(recv_info), 0);
    }
    close(sock_client);
}
