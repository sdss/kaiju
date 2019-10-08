#include <iostream>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <Eigen/Dense>
#include "utils.h"
#include "robotGrid.h"
// #include "betaArm.h"

// define constants


const double pitch = 22.4; // distance to next nearest neighbor

// const double angStep = 1; // degrees
// const int maxPathStepsGlob = (int)(ceil(700.0/angStep));
// line smoothing factor
// const double epsilon =  5 * angStep; // was 7*angStep for 0.1 step size

// const double min_targ_sep = 8; // mm
const double min_targ_sep = 0; // mm

// bool sortTargList(std::array<double, 5> targ1, std::array<double, 5> targ2){
//     return targ1[3] > targ2[3];
// }

bool sortTargList(std::shared_ptr<Target> t1, std::shared_ptr<Target> t2){
    return t1->priority > t2->priority;
}

bool sortRobotPriority(std::shared_ptr<Robot> robot1, std::shared_ptr<Robot> robot2){
    if (!robot1->isAssigned()){
        return true;
    }

    if (!robot2->isAssigned()){
        return false;
    }

    return robot1->assignedTarget->priority > robot2->assignedTarget->priority;
}

bool sortRobotID(std::shared_ptr<Robot> robot1, std::shared_ptr<Robot> robot2){
    return robot1->id < robot2->id;
}

// there is a better way to use the constructor to set attrs....look it up
RobotGrid::RobotGrid(double myAngStep, double myCollisionBuffer, double myEpsilon, int seed){
    // nDia is number of robots along equator of grid
    srand(seed);
    epsilon = myEpsilon;
    collisionBuffer = myCollisionBuffer;
    angStep = myAngStep;
    smoothCollisions = 0;
    maxPathSteps = (int)(ceil(700.0/angStep));
}

void RobotGrid::addRobot(int robotID, double xPos, double yPos, bool hasApogee){
        auto rptr = std::make_shared<Robot>(robotID, xPos, yPos, angStep, hasApogee);
        rptr->setCollisionBuffer(collisionBuffer);
        // Robot robot(ii, xyHexPos(ii, 0), xyHexPos(ii, 1), angStep, betaPair.first, betaPair.second);
        // robot.setCollisionBuffer(collisionBuffer);
        // hack set all alpha betas home
        allRobots.push_back(std::move(rptr));
}

void RobotGrid::addFiducial(double xPos, double yPos){
        std::array<double, 2> fiducial = {xPos, yPos};
        fiducialList.push_back(fiducial);
}

void RobotGrid::initGrid(){
    // sets robots to random points
    double dx, dy, dist;
    nRobots = allRobots.size();
    for (auto r1 : allRobots){
        // add fiducials (potential to collide with)
        for (auto fiducial : fiducialList){
            dx = r1->xPos - fiducial[0];
            dy = r1->yPos - fiducial[1];
            dist = hypot(dx, dy);
            if (dist < pitch+1) { // +1 for numerical buffer
                // std::cout << "add fiducial " << dist << std::endl;
                r1->addFiducial(fiducial);
            }
        }
        for (auto r2 : allRobots){
            // add neighbors (potential to collide with)
            if (r1->id==r2->id){
                continue;
            }
            dx = r1->xPos - r2->xPos;
            dy = r1->yPos - r2->yPos;
            dist = hypot(dx, dy);
            if (dist < (2*pitch+1)){ //+1 for numerical buffer
                // these robots are neighbors
                r1->addNeighbor(r2);
            }
        }
    }

    // ignoring minimum separation for now
    for (auto r : allRobots){
        // r->setXYUniform();
        r->setAlphaBeta(0, 0);
    }
    // ensure robot list is in order of robot id
    std::sort(allRobots.begin(), allRobots.end(), sortRobotID);
}

std::shared_ptr<Robot> RobotGrid::getRobot(int robotInd){
    return allRobots[robotInd];
}

void RobotGrid::setCollisionBuffer(double newBuffer){
    for (auto r : allRobots){
        r->setCollisionBuffer(newBuffer);
    }
}

void RobotGrid::decollide(){
    // iterate over robots and resolve collisions
    // sort by target priority
	  // This segmentation faults if any robot doesn't have
    // an assigned target
    std::sort(allRobots.begin(), allRobots.end(), sortRobotPriority);

    while(getNCollisions()){
        // std::cout << "getting collisions " << std::endl;
        for (auto r : allRobots){
            // std::cout << "decolliding robot " << r.id << std::endl;
            if (r->isCollided()){
                r->decollide();
            }
        }
    }

    std::sort(allRobots.begin(), allRobots.end(), sortRobotID);
}


void RobotGrid::smoothPaths(){
    for (auto r : allRobots){
        r->smoothPath(epsilon);
    }
}

void RobotGrid::verifySmoothed(){
    smoothCollisions = 0;
    for (int ii = 0; ii < nSteps; ii++){
        for (auto r : allRobots){
            r->setAlphaBeta(r->interpSmoothAlphaPath[ii](1), r->interpSmoothBetaPath[ii](1));
            // std::cout << " robot id " << r.id << std::endl;
        }
        smoothCollisions += getNCollisions();
    }
    // std::cout << "interp collisions: " << nCollisions << std::endl;
}



int RobotGrid::getNCollisions(){
    // return number of collisions found
    int nCollide = 0;
    for (auto r : allRobots){
        if (r->isCollided()) {
            nCollide++;
        }
    }
    return nCollide;
}

void RobotGrid::pathGen(){
    // first prioritize robots based on their alpha positions
    // robots closest to alpha = 0 are at highest risk with extended
    // betas for getting locked, so try to move those first
    didFail = true;
    int ii;
    for (ii=0; ii<maxPathSteps; ii++){
        bool allFolded = true;

        for (auto r: allRobots){
            // std::cout << "path gen " << r.betaOrientation.size() << " " << r.betaModel.size() << std::endl;
            // std::cout << "alpha beta " << r.alpha << " " << r.beta << std::endl;
            r->stepTowardFold(ii);
            if (allFolded and r->beta!=180) { // stop when beta = 180} or r.alpha!=0)){
                allFolded = false;
            }
        }
        // std::cout << "------------------" << std::endl;
        // std::cout << "------------------" << std::endl;
        if (allFolded){
            didFail = false;
            break;
        }
        // exit of all robots
    }
    nSteps = ii;
}

void RobotGrid::clearTargetList(){
    targetList.clear();
    // clear all robot target lists
    for (auto r : allRobots){
        r->targetList.clear();
        r->assignedTarget.reset();
    }
}

void RobotGrid::addTargetList(Eigen::MatrixXd myTargetList){
    int nRows = myTargetList.rows();
    // sort by ascending priority 0 first
    // std::sort(myTargetList.begin(), myTargetList.end(), sortTargList);

    // add targets to robots and robots to targets
    for (int ii = 0; ii < nRows; ii++){
        auto targPtr = std::make_shared<Target>((int)myTargetList(ii, 0), myTargetList(ii, 1), myTargetList(ii, 2), (int)myTargetList(ii, 3), (int)myTargetList(ii, 4) );
        targetList.push_back(std::move(targPtr));
        for (auto r : allRobots){
            if (r->isValidTarget(targetList.back())){
                targetList.back()->validRobots.push_back(r);
                r->targetList.push_back(targetList.back());
            }
        }

    }

    // iterate over robots and arrange targets in order of decreasing priority
    for (auto r : allRobots){
        std::sort(r->targetList.begin(), r->targetList.end(), sortTargList);
    }
}

void RobotGrid::setTargetList(Eigen::MatrixXd myTargetList){ //std::vector<std::array<double, 5>> myTargetList){
    // targetID, x, y, priority, fiberID (1=apogee 2=boss)
    // std::array<double, 2> ab;

	clearTargetList();
	addTargetList(myTargetList);
}

void RobotGrid::greedyAssign(){
    // assign the highest priority targets to robots
    // initialize each robot to its highest priority target
    // only allow one target per robot
    for (auto r : allRobots){
        for (auto targ : r->targetList){
            if (targ->isAssigned()){
                // target has been assigned to other robot
                continue;
            }
            targ->assignRobot(r);
            r->assignTarget(targ);
            break;
        }
    }
}

void RobotGrid::pairwiseSwap(){
    // look for pairwise swaps that reduce collisions
    for (auto r1 : allRobots){
        if (r1->isCollided()){
            // use getNCollisions because the swap may
            // decolide the robot, but may introduce a new
            // collision
            double initialCollisions = getNCollisions();
            for (auto r2 : r1->neighbors){
                if (r1->canSwapTarget(r2)){
                    swapTargets(r1, r2);
                    if (initialCollisions <= getNCollisions()){
                        // swap targets back
                        // collision still exists
                        // or is worse!
                        swapTargets(r1, r2);
                    }
                    else {
                        // collision resolved
                        break;
                    }
                }
            }
        }
    }
}

void RobotGrid::swapTargets(std::shared_ptr<Robot> r1, std::shared_ptr<Robot> r2){
    std::shared_ptr<Target> savedTarget = r1->assignedTarget;
    r1->assignTarget(r2->assignedTarget);
    r2->assignTarget(savedTarget);
    // update target->robot assignments
    r1->assignedTarget->assignRobot(r1);
    r2->assignedTarget->assignRobot(r2);
}

std::vector<std::shared_ptr<Robot>> RobotGrid::unassignedRobots(){
    std::vector<std::shared_ptr<Robot>> idleRobos;
    for (auto robot : allRobots){
        if (!robot->isAssigned()){
            idleRobos.push_back(robot);
        }
    }
    return idleRobos;
}

std::vector<std::shared_ptr<Robot>> RobotGrid::targetlessRobots(){
    // robots that don't have any potential targets in target list
    if (targetList.size()==0){
        throw std::runtime_error("targetlessRobots() failure, targetlist not yet set");
    }
    std::vector<std::shared_ptr<Robot>> idleRobos;
    for (auto robot : allRobots){
        if (robot->targetList.size()==0){
            idleRobos.push_back(robot);
        }
    }
    return idleRobos;
}

std::vector<std::shared_ptr<Target>> RobotGrid::unreachableTargets(){
    if (targetList.size()==0){
        throw std::runtime_error("unreachableTargets() failure, target list not yet set");
    }
    std::vector<std::shared_ptr<Target>> badTargs;
    for (auto targ : targetList){
        if (targ->validRobots.size()==0){
            badTargs.push_back(targ);
        }
    }
    return badTargs;
}

std::vector<std::shared_ptr<Target>> RobotGrid::assignedTargets(){
    std::vector<std::shared_ptr<Target>> assignedTargs;
    for (auto r : allRobots){
        if (r->assignedTarget){
            assignedTargs.push_back(r->assignedTarget);
        }
    }
    return assignedTargs;
}

bool RobotGrid::isValidRobotTarget(int robotInd, int targID){
    auto robot = allRobots[robotInd];
    for (auto target : robot->targetList){
        if (target->id == targID and !target->isAssigned()){
					return(true);
        }
    }
    return(false);
}

void RobotGrid::assignRobot2Target(int robotInd, int targID){
    auto robot = allRobots[robotInd];
    bool targetValid = false;
    for (auto target : robot->targetList){
        if (target->id == targID and !target->isAssigned()){
            robot->assignTarget(target);
            target->assignRobot(robot);
            targetValid = true;
            break;
        }
    }
    if (!targetValid){
			for (auto target : robot->targetList){
        if (target->id == targID and target->isAssigned()){
           throw std::runtime_error("assignRobot2Target() failure, targetID already assigned");
        }
			}
        throw std::runtime_error("assignRobot2Target() failure, targetID not valid for robot");
    }
}

void RobotGrid::optimizeTargets(){
    // rewrite!!!
}

// void RobotGrid::optimizeTargets(){
//     // warning...only considering boss fibers
//     // at the moment
//     double myX,myY,nX,nY;
//     bool swapWorked;
//     // Robot * rn;
//     // int randIndex;
//     // int randNeighborIndex;

//     // could compile all this stuff into lookup
//     // tables but this is working ok

//     for (auto r : allRobots){
//         if (!r->isCollided()){
//             continue;  // this robot isn't collided skip it
//         }
//         swapWorked = false;
//         // save current assignment
//         myX = r->bossFiberPos(0);
//         myY = r->bossFiberPos(1);
//         std::vector<int> swappableNeighbors;
//         for (int ii=0; ii < r->neighbors.size(); ii++){
//             auto rn = r->neighbors[ii];
//             nX = rn->bossFiberPos(0);
//             nY = rn->bossFiberPos(1);
//             if (!rn->checkFiberXYGlobal(myX, myY, 2)){
//                 // neighbor can't reach mine
//                 // move to next robot
//                 continue;
//             }
//             if (!r->checkFiberXYGlobal(nX, nY, 2)){
//                 // i cant reach neighbor
//                 // move to next robot
//                 continue;
//             }
//             // we can reach eachother, test it
//             swappableNeighbors.push_back(ii);
//             r->setFiberXY(nX, nY, 2);
//             rn->setFiberXY(myX, myY, 2);
//             if (!r->isCollided()){
//                 // no longer collided
//                 // keep this swap!
//                 // first one that's good
//                 swapWorked = true;
//                 break;
//             }
//             // swap them back
//             r->setFiberXY(myX, myY, 2);
//             rn->setFiberXY(nX, nY, 2);
//         }

//         // if (!swapWorked and swappableNeighbors.size()>0){
//         //     // we didn't find any swap that got rid
//         //     // of collision, pick a random valid
//         //     // swap for this robot
//         //     // set them back,
//         //     // swap didn't work
//         //     randIndex = rand() % swappableNeighbors.size();
//         //     rn = r.neighbors[swappableNeighbors[randIndex]];
//         //     nX = rn->fiber_XYZ(0);
//         //     nY = rn->fiber_XYZ(1);

//         //     r.setFiberXY(nX, nY);
//         //     rn->setFiberXY(myX, myY);
//         // }


//     }


// }
