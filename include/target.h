# pragma once
#include "robot.h"
// #include "robotGrid.h"

class Robot; // defined in robot.h

class Target{
public:
    int id;
    int priority,fiberID;
    float x,y;
    std::vector<std::shared_ptr<Robot>> validRobots;
    Target(int myId, double myX, double myY, int myPriority, int myFiberID);
};
