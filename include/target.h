# pragma once
#include "robot.h"
// #include "robotGrid.h"

class Robot; // defined in robot.h

class Target{
public:
    int assignedRobotID = -1; // ID of robot assigned to this target
    int id;
    int priority,fiberID;
    float x,y;
    std::vector<int> validRobotIDs; // robots that can reach this target
    Target(int myId, double myX, double myY, int myPriority, int myFiberID);
    void assignRobot(int robotID);
    void clearAssignment();
    bool isAssigned();
};
