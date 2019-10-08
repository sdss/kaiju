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

void Target::assignRobot(std::shared_ptr<Robot> robot){
    // should i reset the shared pointer first?
    // assignedRobot.reset()
    return;
    // assignedRobot = robot;
}

bool Target::isAssigned(){
    return false;
    // return (bool)assignedRobot;
}


