#ifndef UNITTEST_H
#define UNITTEST_H

#include"vehicle.h"
#include "savedata.h"
#include <cmath>

class Test
{
public:
    int checkRefPoint(int curr_point_index, double vehicle_pos_x, ReferenceLine& refer_line);
    void lonSolverUnitTest();
    void latSolverUnitTest();
};


#endif // UNITTEST_H