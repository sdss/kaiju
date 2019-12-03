# pragma once

class Fiducial{
public:
    int id;
    double x,y;
    double collisionBuffer;
    Fiducial(int id, double x, double y, double collisionBuffer = 1.5);
};
