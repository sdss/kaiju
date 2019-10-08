#include <iostream>
#include "target.h"


Target::Target(int myID, double myX, double myY, int myPriority, int myFiberID){
    std:: cout << "target created" << std::endl;
    id = myID;
    x = myX;
    y = myY;
    priority = myPriority;
    fiberID = myFiberID;
}

void Target::assignRobot(int robotInd){
    assignedRobotInd = robotInd;
}

bool Target::isAssigned(){
    return assignedRobotInd != 0;
}


