#include <iostream>
#include "target.h"


Target::Target(int id, double x, double y, int fiberID, int priority)
    : id(id), x(x), y(y), priority(priority), fiberID(fiberID)
{}

void Target::clearAssignment(){
    assignedRobotID = -1;
}

void Target::assignRobot(int robotID){
    // make sure robotID is in validRobotID list
    if (std::count(validRobotIDs.begin(), validRobotIDs.end(), robotID)){
        throw std::runtime_error("robotID is not valid for this target.");
    }
    assignedRobotID = robotID;
}

bool Target::isAssigned(){
    return assignedRobotID != -1;
}


