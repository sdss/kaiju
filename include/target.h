# pragma once
#include <vector>
// #include "robot.h"
// #include "robotGrid.h"

// class Robot; // defined in robot.h

enum FiberType {MetrologyFiber, ApogeeFiber, BossFiber}; // order is important

class Target{
public:
    int assignedRobotID = -1; // ID of robot assigned to this target
    int id;
    int priority;
    FiberType fiberType;
    double x,y;
    std::vector<int> validRobotIDs; // robots that can reach this target
    Target(int id, double x, double y, FiberType fiberType, int priority=0);
    void assignRobot(int robotID);
    void clearAssignment();
    bool isAssigned();
};
