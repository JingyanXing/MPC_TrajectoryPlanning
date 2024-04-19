#include "vehicle.h"

Vehicle::Vehicle(point pos, double speed, double acc, double curise_speed, 
                double headingAngle, double headingAngleRate, double wheelAngle, ROAD_MAP& map){
    this->pos_c = pos;
    this->speed = speed;
    this->acc = acc;
    this->jerk = 0;
    this->curise_speed = curise_speed;
    this->expect_speed = curise_speed;
    this->headingAngle = headingAngle;
    this->headingAngleRate = headingAngleRate;
    this->wheelAngle = wheelAngle;
    this->map = map;
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
    this->obstacle_in_lane1.clear();
    this->obstacle_in_lane2.clear();

    // 查找通讯范围内的障碍物
    for(auto& obs : this->map.static_obstacle){
        // 如果在通讯范围内
        if(obs.pos.x <= this->pos_c.x + this->sensoryRange && obs.pos.x >= this->pos_c.x){
            this->obstacle_in_range.emplace_back(obs);
            if(this->checkLaneID(obs.pos) == 1) this->obstacle_in_lane1.emplace_back(obs);
            else this->obstacle_in_lane2.emplace_back(obs);
        }
    }

    // 动态障碍物位置实时刷新
    for(auto& obs : this->map.dynamic_obstacle){
        // 如果在通讯范围内
        if(obs.pos.x <= this->pos_c.x + this->sensoryRange && obs.pos.x >= this->pos_c.x){
            this->obstacle_in_range.emplace_back(obs);
            if(this->checkLaneID(obs.pos) == 1) this->obstacle_in_lane1.emplace_back(obs);
            else this->obstacle_in_lane2.emplace_back(obs);
        }
    }

    // 按x方向位置排序
    sort(this->obstacle_in_range.begin(), this->obstacle_in_range.end(), [](Obstacle a, Obstacle b){
        return a.pos.x < b.pos.x;
    });
    sort(this->obstacle_in_lane1.begin(), this->obstacle_in_lane1.end(), [](Obstacle a, Obstacle b){
        return a.pos.x < b.pos.x;
    });
    sort(this->obstacle_in_lane2.begin(), this->obstacle_in_lane2.end(), [](Obstacle a, Obstacle b){
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
    double expect_following_distance = 20 + std::max(0., this->speed * 1.2 + 
                                                    0.5 * this->speed * (this->speed - front_vehicle_speed) / std::pow(this->MAX_ACCELERATION * 3, 0.5));
    double acc = this->MAX_ACCELERATION * (1 - std::pow(this->speed / (this->curise_speed + 0.001), 4) 
                                            - std::pow(expect_following_distance / (front_vehicle_pos.x - this->pos_c.x + 0.001), 2));
    return (this->speed + acc * 2) >= 0 ? (this->speed + acc * 2) : 0;
}


void Vehicle::InsertLaneFollowPoint(double length){
    int num = (int)(length / 0.1);
    for(int i = 0; i < num; i++){
        point tmp(this->refer_line.back().x + 0.1, this->refer_line.back().y);
        this->refer_line.emplace_back(tmp);
    }
}


void Vehicle::InsertLaneChangePoint(point start_point, point end_point){
    int res = int((end_point.x - start_point.x - rear_buffer) / point_gap);
    double delta_y = (end_point.y - start_point.y) / res;
    for(int i = 1; i <= res; i++){
        point tmp(start_point.x + point_gap * i, start_point.y + delta_y * i);
        this->refer_line.emplace_back(tmp);
    }
}


void Vehicle::updateReferenceLine(){
    int lane_id = this->checkLaneID(this->pos_c);
    double line_length = this->speed * 2 + 30;

    //处于巡航状态或不需要换道,只检查当前车道前方障碍物
    if(this->is_curise || !this->state){
        this->refer_line.clear();
        point tmp(this->pos_c.x, 1.75);
        if(lane_id == 2) tmp.y = 5.25;
        // 存在障碍物且间距小于line_length，更新line_length长度
        if(lane_id == 1 && this->obstacle_in_lane1.size() != 0 && this->obstacle_in_lane1[0].pos.x - this->pos_c.x  + 10 < line_length)
            line_length = this->obstacle_in_lane1[0].pos.x - this->pos_c.x - 10;
        if(lane_id == 2 && this->obstacle_in_lane2.size() != 0 && this->obstacle_in_lane2[0].pos.x - this->pos_c.x  + 10 < line_length)
            line_length = this->obstacle_in_lane2[0].pos.x - this->pos_c.x - 10;
        this->refer_line.emplace_back(tmp);
        this->InsertLaneFollowPoint(line_length);
    }
    //切换目标车道中心线为参考线,换道期间只更新一次
    else if(this->state == 1){
        this->state++;
        this->refer_line.clear();
        point tmp(this->pos_c.x, 1.75);
        point end_point;
        if(this->target_lane == 1) tmp.y = 5.25;
        // 存在障碍物且间距小于line_length，更新line_length长度
        if(this->target_lane == 1){
            if(this->obstacle_in_lane1.size() != 0 && this->obstacle_in_lane1[0].pos.x - this->pos_c.x  + 10 < line_length){
                line_length = this->obstacle_in_lane1[0].pos.x - this->pos_c.x - 10;
            }
            end_point.x = this->obstacle_in_lane2[0].rear_left.x;
            end_point.y = 1.75;
        }
        if(this->target_lane == 2){
            if(this->obstacle_in_lane2.size() != 0 && this->obstacle_in_lane2[0].pos.x - this->pos_c.x  + 10 < line_length){
                line_length = this->obstacle_in_lane2[0].pos.x - this->pos_c.x - 10;
            }
            end_point.x = this->obstacle_in_lane1[0].rear_left.x;
            end_point.y = 5.25;
        }
        this->refer_line.emplace_back(tmp);
        this->InsertLaneChangePoint(this->refer_line.back(), end_point);
        double res_line_length = line_length - this->refer_line.back().x + this->refer_line.front().x;
        res_line_length = res_line_length < 10 ? 10 : res_line_length;
        this->InsertLaneFollowPoint(res_line_length);
        
    }
    // 换道期间参考线更新
    else{
        double length = this->pos_c.x - this->refer_line.front().x;
        while(this->refer_line.front().x < this->pos_c.x){
            this->refer_line.erase(this->refer_line.begin());
        }
        if(this->target_lane == 1 && this->obstacle_in_lane1.size() != 0){
            length = std::min(length, this->obstacle_in_lane1[0].rear_left.x - this->refer_line.back().x - 5);
        }
        if(this->target_lane == 2 && this->obstacle_in_lane2.size() != 0){
            length = std::min(length, this->obstacle_in_lane2[0].rear_left.x - this->refer_line.back().x - 5);
        }
        this->InsertLaneFollowPoint(length);
    }
    
}


void Vehicle::setCuriseMode(){
    this->is_curise = true;
}


void Vehicle::setLatMpcMode(int mode){
    this->lat_solver_mode = mode;
}


int Vehicle::checkRefPoint(double vehicle_pos_x){
    int res = 0;
    while(vehicle_pos_x > this->refer_line[res].x){
        res++;
    }
    return res;
}


void Vehicle::drive(){
    this->GetObstacleInSensoryRange();
    // 非巡航状态下，前方静止障碍物或运动速度小于给定curise_speed，TTC小于阈值，且目标车道没有障碍物进入换道状态
    int lane_id = this->checkLaneID(this->pos_c);
    if (!this->state && !this->is_curise) {
      if (lane_id == 1 && this->obstacle_in_lane1.size() != 0 &&
          (!this->obstacle_in_lane1[0].is_dynamic ||
           this->obstacle_in_lane1[0].speed < this->curise_speed)) {
        double TTC = (this->obstacle_in_lane1[0].pos.x - this->pos_c.x) /
                     (this->speed - this->obstacle_in_lane1[0].speed + 0.001);
        if (0 <= TTC && TTC <= 5 + 0.6 * this->speed &&
            (this->obstacle_in_lane2.size() == 0 ||
             this->obstacle_in_lane2[0].pos.x >
                 this->obstacle_in_lane1[0].pos.x)) {
          this->state = 1;
          this->target_lane = 2;
        }
        if (this->obstacle_in_lane1.size() != 0 &&
            !this->obstacle_in_lane1[0].is_dynamic &&
            this->obstacle_in_lane1[0].rear_left.x - this->pos_c.x < 35) {
          this->state = 1;
          this->target_lane = 2;
        }
      }
      if (lane_id == 2 && this->obstacle_in_lane2.size() != 0 &&
          (!this->obstacle_in_lane2[0].is_dynamic ||
           this->obstacle_in_lane2[0].speed < this->curise_speed)) {
        double TTC = (this->obstacle_in_lane2[0].pos.x - this->pos_c.x) /
                     (this->speed - this->obstacle_in_lane2[0].speed + 0.001);
        if (0 <= TTC && TTC <= 5 + 0.6 * this->speed &&
            (this->obstacle_in_lane1.size() == 0 ||
             this->obstacle_in_lane1[0].pos.x >
                 this->obstacle_in_lane2[0].pos.x)) {
          this->state = 1;
          this->target_lane = 1;
        }
        if (this->obstacle_in_lane2.size() != 0 &&
            !this->obstacle_in_lane2[0].is_dynamic &&
            this->obstacle_in_lane2[0].rear_left.x - this->pos_c.x < 35) {
          this->state = 1;
          this->target_lane = 1;
        }
      }
    }
    this->updateReferenceLine();
    // 更新expect_speed
    // 跟车采用idm模型生成expect_speed,换道过程保持初始速度
    if(this->state == 0 || this->is_curise){
        if(lane_id == 1){
            if(this->obstacle_in_lane1.size() == 0) this->expect_speed = this->curise_speed;
            else this->expect_speed = this->IDMBasedSpeed(obstacle_in_lane1[0].speed, obstacle_in_lane1[0].pos);
        }
        if(lane_id == 2){
            if(this->obstacle_in_lane2.size() == 0) this->expect_speed = this->curise_speed;
            else this->expect_speed = this->IDMBasedSpeed(obstacle_in_lane2[0].speed, obstacle_in_lane2[0].pos);
        }
    }
    else{
        if(this->expect_speed == 0) this->expect_speed = 1;
    }
    // 更新车辆位置
    //基于当前位置向前查找参考点
    std::vector<std::vector<double>> refer_point;
    for(int i = 1; i < 21; i++){
        // 向前搜索参考线上参考点索引
        int index = this->checkRefPoint(this->pos_c.x + this->expect_speed * i / 10);
        refer_point.push_back({this->refer_line[index].x, 
                               this->refer_line[index].y,
                               this->expect_speed,
                               (double)atan2((this->refer_line[index].y - this->refer_line[index - 1].y),
                                             (this->refer_line[index].x - this->refer_line[index - 1].x))});
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
    if(!this->state){
        if(lane_id == 1){
            lower_l.resize(20, this->map.boundary1[0].y);
            upper_l.resize(20, this->map.boundary2[0].y);
        }
        else{
            lower_l.resize(20, this->map.boundary2[0].y);
            upper_l.resize(20, this->map.boundary3[0].y);
        }
    }
    else{
        lower_l.resize(20, this->map.boundary1[0].y);
        upper_l.resize(20, this->map.boundary3[0].y);
    }

    //横向mpc求解器
    if(this->lat_solver_mode == 0){
      casadi::DM heading_angle_rate_dot = LatMpcSolver(
          this->pos_c.x, this->pos_c.y, this->headingAngle,
          this->headingAngleRate, refer_point.back()[0], refer_point.back()[1],
          refer_point.back()[3],
          10 * (refer_point.back()[3] - refer_point[18][3]), ref_speed, lower_l,
          upper_l, this->MAX_WHEELANGLE, this->MAX_HEADINGANGLE,
          this->MAX_SPEED, this->wheelBase);
      // 更新规划轨迹
      std::vector<double> tmp_state = {
          this->pos_c.x, this->pos_c.y,      this->speed,
          this->acc,     this->headingAngle, this->headingAngleRate};
      this->planning_trajectory.clear();
      for (int i = 1; i <= 20; i++) {
        tmp_state[0] += tmp_state[2] * 0.1 * cos(tmp_state[4]);
        tmp_state[1] += tmp_state[2] * 0.1 * sin(tmp_state[4]);
        tmp_state[2] += tmp_state[3] * 0.1;
        tmp_state[3] += (double)j(i - 1);
        tmp_state[4] += tmp_state[5] * 0.1;
        tmp_state[5] += (double)heading_angle_rate_dot(i - 1) * 0.1;
        this->planning_trajectory += std::to_string(tmp_state[0]) + ',' +
                                     std::to_string(tmp_state[1]) + ',';
        }
        this->planning_trajectory.pop_back();
        //更新车辆状态
        this->pos_c.x += this->speed * 0.1 * cos(this->headingAngle);
        this->pos_c.y += this->speed * 0.1 * sin(this->headingAngle);
        this->headingAngle += this->headingAngleRate * 0.1;
        this->headingAngleRate += (double)heading_angle_rate_dot(0) * 0.1;
        this->wheelAngle = (double)atan2((double)heading_angle_rate_dot(0) * this->wheelBase, this->speed + 0.001); //避免速度为0
    }
    else if(this->lat_solver_mode == 1){
      casadi::DM wheel_angle_dot = LatMpcSolverWheelAngle(
          this->pos_c.y, this->speed * sin(this->headingAngle),
          this->headingAngle, this->headingAngleRate, this->wheelAngle,
          refer_point, ref_speed, lower_l, upper_l, this->MAX_WHEELANGLE,
          this->MAX_WHEELANGLE_RATE, this->MAX_HEADINGANGLE, this->MAX_SPEED,
          this->wheelBase, this->weight, this->front_wheel_corner_stiffness,
          this->rear_wheel_corner_stiffness, this->moment_of_inertia);
      // 更新规划轨迹
      std::vector<double> tmp_state = {
          this->pos_c.x,   this->pos_c.y,      this->speed,
          this->acc,       this->headingAngle, this->headingAngleRate,
          this->wheelAngle};
      this->planning_trajectory.clear();
      for (int i = 1; i <= 20; i++) {
        tmp_state[5] +=
            tmp_state[2] * tan(tmp_state[6]) / this->wheelBase * 0.1;
        tmp_state[0] += tmp_state[2] * 0.1 * cos(tmp_state[4]);
        tmp_state[1] += tmp_state[2] * 0.1 * sin(tmp_state[4]);
        tmp_state[2] += tmp_state[3] * 0.1;
        tmp_state[3] += (double)j(i - 1);
        tmp_state[4] += tmp_state[5] * 0.1;
        tmp_state[6] += (double)wheel_angle_dot(i - 1) * 0.1;
        this->planning_trajectory += std::to_string(tmp_state[0]) + ',' +
                                     std::to_string(tmp_state[1]) + ',';
        }
        this->planning_trajectory.pop_back();
        //更新车辆状态
        this->pos_c.x += this->speed * 0.1 * cos(this->headingAngle);
        this->pos_c.y += this->speed * 0.1 * sin(this->headingAngle);
        this->headingAngle += this->headingAngleRate * 0.1;
        this->headingAngleRate = this->speed * tan(this->wheelAngle) / this->wheelBase * 0.1;
        this->wheelAngle += (double)wheel_angle_dot(0) * 0.1; 
    }
    this->headingAngle = std::min(this->headingAngle, this->MAX_HEADINGANGLE);
    this->headingAngle = std::max(this->headingAngle, -this->MAX_HEADINGANGLE);
    this->wheelAngle = std::min(this->wheelAngle, this->MAX_WHEELANGLE);
    this->wheelAngle = std::max(this->wheelAngle, -this->MAX_WHEELANGLE);
    this->speed += this->acc * 0.1;
    this->acc += (double)j(0);
    this->saveVehicleState();
    this->map.updateDynamicObstacle();
    // 结束换道
    if(this->state == 2){
        if(this->target_lane == 1 && abs(this->pos_c.y - 1.75) < 0.1){
            this->state = 0;
            this->target_lane = -1;
        }
        if(this->target_lane == 2 && abs(this->pos_c.y - 5.25) < 0.1){
            this->state = 0;
            this->target_lane = -1;
        }
    }
    // std::this_thread::sleep_for(std::chrono::seconds(1));
}

