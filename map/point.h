#ifndef POINT_H
#define POINT_H

struct point{
    double x;
    double y;
    point(double _x,double _y) : x(_x), y(_y){};
    point() : x(0), y(0){};
};
#endif // POINT_H