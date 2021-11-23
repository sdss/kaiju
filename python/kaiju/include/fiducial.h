# pragma once
#include "coordio.h"

class Fiducial{
public:
    int id;
    double x,y;
    vec3 xyzWok;
    double collisionBuffer;
    Fiducial(int id, vec3 xyzWok, double collisionBuffer = 2.5);
};
