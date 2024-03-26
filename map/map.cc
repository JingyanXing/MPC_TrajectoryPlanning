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


void LinearInsertPoint(std::vector<point>& refer_line, point start_point, point end_point){
    int res = int((end_point.x - start_point.x - rear_buffer) / point_gap);
    double delta_y = (end_point.y - start_point.y) / res;
    for(int i = 1; i <= res; i++){
        point tmp(start_point.x + point_gap * i, start_point.y + delta_y * i);
        refer_line.emplace_back(tmp);
    }
    refer_line.back().y = end_point.y;
}

void InsertPoint(std::vector<point>& refer_line)
{
    point tmp(refer_line.back().x + point_gap, refer_line.back().y);
    refer_line.emplace_back(tmp);
}


void ReferenceLine::update(point pos, int refer_line_index, double width, std::vector<point>& refer_line, ROAD_MAP& map){
    if(refer_line_index != 0) refer_line.erase(refer_line.begin(), refer_line.begin() + refer_line_index);
    double buffer = 35; // 换道缓冲距离
    double tmp_y;
    if(abs(tmp_y - map.middleline1[0].y) <= abs(tmp_y - map.middleline2[0].y)) tmp_y = map.middleline1[0].y;
    else tmp_y = map.middleline2[0].y;
    std::vector<Obstacle> obstacle_in_range; //存储感知范围内所有障碍物，无论动态还是静态
    // 查找通讯范围内的障碍物
    for(auto& obs : map.static_obstacle){
        // 如果在通讯范围内且没有被搜索过
        if(obs.pos.x <= pos.x + this->sensoryRange && this->obstacle_set.find(obs.name) == this->obstacle_set.end()){
            obstacle_in_range.emplace_back(obs);
            this->obstacle_set.insert(obs.name);
        }
        if(obs.pos.x > pos.x + this->sensoryRange) break;
    }

    for(auto& obs : map.dynamic_obstacle){
        // 如果在通讯范围内且没有被搜索过
        if(obs.pos.x <= pos.x + this->sensoryRange && this->obstacle_set.find(obs.name) == this->obstacle_set.end()){
            obstacle_in_range.emplace_back(obs);
            this->obstacle_set.insert(obs.name);
        }
        if(obs.pos.x > pos.x + this->sensoryRange) break;
    }
    // 按x方向位置排序
    sort(obstacle_in_range.begin(), obstacle_in_range.end(), [](Obstacle a, Obstacle b){
        return a.pos.x < b.pos.x;
    });
    int obstacle_index = 0;
    if(refer_line.size() == 0) refer_line.push_back({pos.x, tmp_y});
    //参考线延长至150m，如遇到障碍物，为确保绕过，会适当延长,确保参考线的back().y在道路中心线
    while(refer_line.back().x <= pos.x + this->sensoryRange){
        //无障碍或障碍物已经遍历完
        if(obstacle_index == obstacle_in_range.size()){
            // std::cout << "no obstacle in range" << std::endl;
            InsertPoint(refer_line);
            continue;
        }
        //未到达障碍物前
        if(refer_line.back().x + buffer < obstacle_in_range[obstacle_index].rear_left.x) InsertPoint(refer_line);
        else{
            //检查障碍物对于车辆行驶是否存在干扰
            //不存在干扰
            // std::cout << "no interference" << std::endl;
            if(abs(obstacle_in_range[obstacle_index].pos.y - refer_line.back().y) > (width + obstacle_in_range[obstacle_index].width) / 2){
                obstacle_index++;
                InsertPoint(refer_line);
            }
            //存在干扰
            else{
                //静态障碍物
                if(!obstacle_in_range[obstacle_index].is_dynamic){
                    //向右换道
                    if(abs(obstacle_in_range[obstacle_index].pos.y - map.middleline2[0].y) < map.width / 2){
                        point end_point(obstacle_in_range[obstacle_index].rear_left.x, map.middleline1[0].y);
                        LinearInsertPoint(refer_line, refer_line.back(), end_point);
                        obstacle_index++;
                    }
                    //向左换道
                    else{
                        point end_point(obstacle_in_range[obstacle_index].rear_left.x, map.middleline2[0].y);
                        LinearInsertPoint(refer_line, refer_line.back(), end_point);
                        obstacle_index++;
                    }
                }
                //TODO:动态障碍物
            }
        }
    }

}

ReferenceLine::ReferenceLine(point pos, double width, ROAD_MAP& map){
    this->update(pos, 0, width, this->refer_line, map);
    std::cout << "参考线初始化成功" << std::endl;
}

ReferenceLine::ReferenceLine(){};

ReferenceLine::~ReferenceLine(){};