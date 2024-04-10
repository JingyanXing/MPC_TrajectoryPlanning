#ifndef MAP_H
#define MAP_H
#include"obstacle.h"
#include<unordered_set>
#include<algorithm>
#define point_gap 0.1 
#define rear_buffer 5
/*x-y参考系下生成道路
——————————————————————————————————b3
----------------------------------r2
——————————————————————————————————b2
----------------------------------r1
——————————————————————————————————b1
*/


class ROAD_MAP{
public:
    std::vector<point> boundary1; //右侧道路，idx=0行驶方向右边界
    std::vector<point> boundary2;
    std::vector<point> boundary3;
    std::vector<point> middleline1;//道路中心参考线
    std::vector<point> middleline2;
    std::vector<Obstacle> static_obstacle;
    std::vector<Obstacle> dynamic_obstacle;
    double width = 3.5;
    double length;

    //两种生成方式
    //地图初始化时生成静态障碍物数量static_obstacle,生成动态障碍物数量dynamic_obstacle，均为随机生成
    ROAD_MAP(double length, int static_obstacle_num, int dynamic_obstacle_num);
    //初始化时生成指定位置的障碍物,障碍物按x位置升序排序
    ROAD_MAP(double length, std::vector<Obstacle> static_obstacle, std::vector<Obstacle> dynamic_obstacle);
    ROAD_MAP();
    void updateDynamicObstacle();
    //检查是否堵塞
    bool check_block();
    ~ROAD_MAP();
};

class ReferenceLine{
public:
    //当前参考线的生成逻辑较为简单，不能支持短距离多障碍物的连续避让
    //支持短距离内单个静态障碍物避让，动态障碍物跟驰或超车
    double sensoryRange = 150;//车辆感知范围，x方向150m
    std::vector<point> refer_line;
    std::unordered_set<std::string> obstacle_set; //已经搜索过的障碍物加入集合，防止重复搜索

    void update(point pos, double ego_speed, double target_speed, int refer_line_index, double width, std::vector<point>& refer_line, ROAD_MAP& map);//车辆位置，车辆宽度
    ReferenceLine(point pos, double ego_speed, double target_speed, double width, ROAD_MAP& map);
    ReferenceLine();
    ~ReferenceLine();
};

#endif