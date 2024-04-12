#include "vehicle.h"

Vehicle::Vehicle(point pos, double speed, double acc, double expect_speed, 
                double headingAngle, double headingAngleRate, double wheelAngle, ROAD_MAP& map){
    this->pos_c = pos;
    this->speed = speed;
    this->acc = acc;
    this->jerk = 0;
    this->expect_speed = expect_speed;
    this->headingAngle = headingAngle;
    this->headingAngleRate = headingAngleRate;
    this->wheelAngle = wheelAngle;
    this->map = map;
    this->refer_line = ReferenceLine(this->pos_c, this->speed, this->expect_speed, this->width, this->map);
    for(int i = 0; i < 20; i++){
        this->planning_trajectory += std::to_string(pos.x + this->speed * 0.1 * i) + ',' + std::to_string(pos.y) + ',';
    }
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


// 获取感知范围内障碍物，用于可视化
void Vehicle::GetObstacleInSensoryRange(){
    this->obstacle_in_range.clear();
    // 查找通讯范围内的障碍物
    for(auto& obs : this->map.static_obstacle){
        // 如果在通讯范围内
        if(obs.pos.x <= this->pos_c.x + this->sensoryRange && obs.pos.x >= this->pos_c.x){
            this->obstacle_in_range.emplace_back(obs);
        }
    }

    // 动态障碍物位置实时刷新
    for(auto& obs : this->map.dynamic_obstacle){
        // 如果在通讯范围内
        if(obs.pos.x <= this->pos_c.x + this->sensoryRange && obs.pos.x >= this->pos_c.x){
            this->obstacle_in_range.emplace_back(obs);
        }
    }

    // 按x方向位置排序
    sort(this->obstacle_in_range.begin(), this->obstacle_in_range.end(), [](Obstacle a, Obstacle b){
        return a.pos.x < b.pos.x;
    });
}


int Vehicle::checkLaneID(point pos){
    int lane_id;
    if(abs(this->map.middleline1[0].y - pos.y) < abs(this->map.middleline2[0].y - pos.y)) lane_id = 1;
    else lane_id = 2;
    return lane_id;
}

double Vehicle::IDMBasedSpeed(double front_vehicle_speed, point front_vehicle_pos){
    double expect_following_distance = 10 + std::max(0., this->speed * 0.6 + 
                                                    0.5 * this->speed * (this->speed - front_vehicle_speed) / std::pow(this->MAX_ACCELERATION * 3, 0.5));
    double acc = this->MAX_ACCELERATION * (1 - std::pow(this->speed / front_vehicle_speed, 4) 
                                            - std::pow(expect_following_distance / (front_vehicle_pos.x - this->pos_c.x), 2));
    return this->speed + acc * 2;
}


void Vehicle::updateReferenceLine(){
    this->GetObstacleInSensoryRange();
    std::vector<Obstacle> obstacle_in_current_lane; //存储自车所在车道障碍物
    std::vector<Obstacle> obstacle_in_target_lane; //存储另一车道障碍物

    //处于巡航状态,只检查当前车道前方障碍物，基于IDM模型设置target_speed
    if(this->is_curise){
        // 查找所在车道
        int lane_id = this->checkLaneID(this->pos_c);

        for(auto& obs : this->obstacle_in_range){
            if(this->checkLaneID(obs.pos) == lane_id){
                obstacle_in_current_lane.emplace_back(obs);
                break;
            }
        }

        // 没有前车设置目标速度为巡航速度
        if(obstacle_in_current_lane.size() == 0){
            this->expect_speed = this->curise_speed;
        }
        // 存在前车设置目标速度为前车速度
        else{
            this->expect_speed = this->IDMBasedSpeed(obstacle_in_current_lane[0].speed, obstacle_in_current_lane[0].pos);
        }
    }
    else{

    }
    
}


void CruiseControl(ROAD_MAP& m){

}

int Vehicle::checkRefPoint(int curr_point_index, double vehicle_pos_x, ReferenceLine& refer_line){
    int res = curr_point_index;
    while(vehicle_pos_x > refer_line.refer_line[res].x){
        res++;
    }
    return res;
}


void Vehicle::drive(){
    this->GetObstacleInSensoryRange();
    // 更新车辆位置
    //基于当前位置向前查找参考点
    std::vector<std::vector<double>> refer_point;
    for(int i = 1; i < 21; i++){
        // 向前搜索参考线上参考点索引
        int index = this->checkRefPoint(this->refer_line_index, this->pos_c.x + this->expect_speed * i / 10, this->refer_line);
        refer_point.push_back({this->refer_line.refer_line[index].x, 
                               this->refer_line.refer_line[index].y,
                               this->expect_speed,
                               (double)atan2((this->refer_line.refer_line[index].y - this->refer_line.refer_line[index - 1].y),
                                             (this->refer_line.refer_line[index].x - this->refer_line.refer_line[index - 1].x))});
    }
    //纵向mpc求解器
    casadi::DM j = LonMpcSolver(this->pos_c.x, this->speed, this->acc, refer_point.back()[0], 
                                refer_point.back()[2], 0, this->MAX_SPEED, this->MAX_ACCELERATION, 
                                this->MAX_DECELERATION, this->MAX_JERK);
    //更新参考速度
    std::vector<double> ref_speed = {this->speed + this->acc * 0.1};
    double tmp_acc = this->acc;
    for(int x = 0; x < 19; x++){
        tmp_acc += (double)j(x);
        ref_speed.push_back(this->speed + tmp_acc * 0.1);
    }

    //TODO：设置横向位置边界约束,需要修改为动态的
    std::vector<double> lower_l;
    std::vector<double> upper_l;
    if(refer_point[0][1] == this->map.middleline1[0].y){
        lower_l.resize(20, this->map.boundary1[0].y);
        upper_l.resize(20, this->map.boundary2[0].y);
    }
    else if(refer_point[0][1] == this->map.middleline2[0].y){
        lower_l.resize(20, this->map.boundary2[0].y);
        upper_l.resize(20, this->map.boundary3[0].y);
    }
    else{
        lower_l.resize(20, this->map.boundary1[0].y);
        upper_l.resize(20, this->map.boundary3[0].y);
    }

    //横向mpc求解器
    casadi::DM heading_angle_rate_dot = LatMpcSolver(this->pos_c.x, this->pos_c.y, this->headingAngle, this->headingAngleRate, refer_point.back()[0], 
                                                    refer_point.back()[1], refer_point.back()[3], refer_point.back()[3] - refer_point[19][3], 
                                                    ref_speed, lower_l, upper_l, this->MAX_WHEELANGLE, this->MAX_HEADINGANGLE,this->MAX_SPEED, this->wheelBase);
    //更新规划轨迹
    std::vector<double> tmp_state = {this->pos_c.x, this->pos_c.y, this->speed, this->acc, this->headingAngle, this->headingAngleRate};
    this->planning_trajectory.clear();
    for(int i = 1; i <= 20; i++){
        tmp_state[0] += tmp_state[2] * 0.1 * cos(tmp_state[4]);
        tmp_state[1] += tmp_state[2] * 0.1 * sin(tmp_state[4]);
        tmp_state[2] += tmp_state[3] * 0.1;
        tmp_state[3] += (double)j(i - 1);
        tmp_state[4] += tmp_state[5] * 0.1;
        tmp_state[5] += (double)heading_angle_rate_dot(i - 1) * 0.1;
        this->planning_trajectory += std::to_string(tmp_state[0]) + ',' + std::to_string(tmp_state[1]) + ',';
    }
    this->planning_trajectory.pop_back();
    //更新车辆状态
    this->pos_c.x += this->speed * 0.1 * cos(this->headingAngle);
    this->pos_c.y += this->speed * 0.1 * sin(this->headingAngle);
    this->headingAngle += this->headingAngleRate * 0.1;
    this->headingAngleRate += (double)heading_angle_rate_dot(0) * 0.1;
    this->wheelAngle = (double)atan2((double)heading_angle_rate_dot(0) * this->wheelBase, this->speed + 0.001); //避免速度为0
    this->speed += this->acc * 0.1;
    this->acc += (double)j(0);
    this->saveVehicleState();
    int new_refer_line_index = this->refer_line_index;
    while(refer_line.refer_line[new_refer_line_index].x < this->pos_c.x) new_refer_line_index++;
    this->refer_line_index = new_refer_line_index;
    this->map.updateDynamicObstacle();


    if(this->refer_line_index > 30){
        this->refer_line.update(this->pos_c, this->speed, this->expect_speed, this->refer_line_index, this->width, this->refer_line.refer_line, this->map);
        this->refer_line_index = 0;
    }
}

