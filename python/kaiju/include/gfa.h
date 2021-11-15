# pragma once
#include "coordio.h"

class GFA{
public:
    int id;
    std::array<vec3, 2> collisionSegWokXYZ;
    double collisionBuffer;
    GFA(int id, std::array<vec3, 2> collisionSegWokXYZ, double collisionBuffer = 2.5);
};
