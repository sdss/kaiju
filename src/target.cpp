#include "target.h"


Target::Target(int myID, double myX, double myY, int myPriority, int myFiberID){
    id = myID;
    x = myX;
    y = myY;
    priority = myPriority;
    fiberID = myFiberID;
}
