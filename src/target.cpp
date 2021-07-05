#include <stdexcept>
#include <algorithm>
#include <iostream>
#include "target.h"


Target::Target(long id, vec3 xyzWok, FiberType fiberType, int priority)
    : id(id), xyzWok(xyzWok), priority(priority), fiberType(fiberType)
{
    x = xyzWok[0];
    y = xyzWok[1];
    z = xyzWok[2];
}

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


