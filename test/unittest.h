#ifndef UNITTEST_H
#define UNITTEST_H

#include"vehicle.h"
#include "savedata.h"
#include <unistd.h>
#include <cmath>
#include <arpa/inet.h>
#include <cstring>

class Test
{
public:
    int checkRefPoint(int curr_point_index, double vehicle_pos_x);
    void lonSolverUnitTest();
    void latSolverUnitTest();
    void run(Vehicle& vehicle, int step);
};


#endif // UNITTEST_H