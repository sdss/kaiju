# pragma once
#include "robot.h"
// #include "robotGrid.h"

class Robot; // defined in robot.h

class Target{
public:
    int assignedRobotInd = -1; // index in RobotGrid.allRobots
    int id;
    int priority,fiberID;
    float x,y;
    std::vector<int> robotInds;
    Target(int myId, double myX, double myY, int myPriority, int myFiberID);
    void assignRobot(int robotInd);
    bool isAssigned();
};
