#include <stdexcept>
#include <algorithm>
#include <iostream>
#include "target.h"


Target::Target(int id, double x, double y, FiberType fiberType, int priority)
    : id(id), x(x), y(y), priority(priority), fiberType(fiberType)
{}

void Target::clearAssignment(){
    assignedRobotID = -1;
}

void Target::assignRobot(int robotID){
    // make sure robotID is in validRobotID list
    int ii = std::count(validRobotIDs.begin(), validRobotIDs.end(), robotID);
    if (ii == 0){
        throw std::runtime_error("robotID is not valid for this target.");
    }
    assignedRobotID = robotID;
}

bool Target::isAssigned(){
    return assignedRobotID != -1;
}


