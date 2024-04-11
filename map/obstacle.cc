#include"obstacle.h"


Obstacle::Obstacle(int type, point pos, std::string name){
    this->pos = pos;
    this->name = name;
    
    switch(type){
        case 0:
            //小轿车
            this->length = 5;
            this->width = 1.9;
            break;
        case 1:
            //货车
            this->length = 10;
            this->width = 2.3;
            break;
        case 2:
            //非机动车
            this->length = 1.8;
            this->width = 0.5;
            break;
    }
    this->front_right.x = pos.x + this->length / 2;
    this->front_right.y = pos.y - this->width / 2;
    this->rear_right.x = pos.x - this->length / 2;
    this->rear_right.y = pos.y - this->width / 2;
    this->rear_left.x = pos.x - this->length / 2;
    this->rear_left.y = pos.y + this->width / 2;
    this->front_left.x = pos.x + this->length / 2;
    this->front_left.y = pos.y + this->width / 2;
}

Obstacle::Obstacle(int type, point pos, std::string name, double speed){
    this->pos = pos;
    this->is_dynamic = true;
    this->speed = speed;
    this->name = name;
    switch(type){
        case 0:
            //小轿车
            this->length = 5;
            this->width = 1.9;
            break;
        case 1:
            //货车
            this->length = 10;
            this->width = 2.3;
            break;
        case 2:
            //非机动车
            this->length = 1.8;
            this->width = 0.5;
            break;
    }
    this->front_right.x = pos.x + this->length / 2;
    this->front_right.y = pos.y - this->width / 2;
    this->rear_right.x = pos.x - this->length / 2;
    this->rear_right.y = pos.y - this->width / 2;
    this->rear_left.x = pos.x - this->length / 2;
    this->rear_left.y = pos.y + this->width / 2;
    this->front_left.x = pos.x + this->length / 2;
    this->front_left.y = pos.y + this->width / 2;
}
