#include <iostream>
#include "target.h"


Target::Target(int myID, double myX, double myY, int myPriority, int myFiberID){
    // std:: cout << "target created" << std::endl;
    id = myID;
    x = myX;
    y = myY;
    priority = myPriority;
    fiberID = myFiberID;
}

void Target::clearAssignment(){
    assignedRobotID = -1;
}

void Target::assignRobot(int robotID){
    // make sure robotID is in validRobotID list
    // could use find instead?
    if (std::count(validRobotIDs.begin(), validRobotIDs.end(), robotID)){
        throw std::runtime_error("robotID is not valid for this target.");
    }
    assignedRobotID = robotID;
}

bool Target::isAssigned(){
    return assignedRobotID != -1;
}


