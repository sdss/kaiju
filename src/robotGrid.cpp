#include <iostream>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <Eigen/Dense>
#include "utils.h"
#include "robotGrid.h"
// #include "betaArm.h"

// define constants
const double betaCollisionRadius = 1.5; // mm (3mm wide)

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

bool reverseSortTargList(std::shared_ptr<Target> t1, std::shared_ptr<Target> t2){
    return t1->priority < t2->priority;
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

    // first sort robots by robotID
    std::sort(allRobots.begin(), allRobots.end(), sortRobotID);

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
        int r2Ind = -1;
        for (auto r2 : allRobots){
            r2Ind++;
            // add neighbors (potential to collide with)
            if (r1->id==r2->id){
                continue;
            }
            dx = r1->xPos - r2->xPos;
            dy = r1->yPos - r2->yPos;
            dist = hypot(dx, dy);
            if (dist < (2*pitch+1)){ //+1 for numerical buffer
                // these robots are neighbors
                r1->addNeighbor(r2Ind);
            }
        }
    }

    // ignoring minimum separation for now
    for (auto r : allRobots){
        // r->setXYUniform();
        r->setAlphaBeta(0, 0);
    }

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

    // decollide robots, throwing out lowest priority targets first


    // std::sort(allRobots.begin(), allRobots.end(), sortRobotPriority);

    while(getNCollisions()){
        // first try moving unassigned robots
        for (auto r : targetlessRobots()){
            if (isCollided(r)){
                decollideRobot(r);
            }
        }

        // next start throwing out lowest priority targets
        auto targets = assignedTargets();
        std::sort(targets.begin(), targets.end(), reverseSortTargList);
        for (auto t : targets){
            auto r = allRobots[t->assignedRobotInd];
            if (isCollided(r)){
                decollideRobot(r);
            }
        }
    }
}


// void RobotGrid::smoothPaths(){
//     for (auto r : allRobots){
//         r->smoothPath(epsilon);
//     }
// }

// void RobotGrid::verifySmoothed(){
//     smoothCollisions = 0;
//     for (int ii = 0; ii < nSteps; ii++){
//         for (auto r : allRobots){
//             r->setAlphaBeta(r->interpSmoothAlphaPath[ii](1), r->interpSmoothBetaPath[ii](1));
//             // std::cout << " robot id " << r.id << std::endl;
//         }
//         smoothCollisions += getNCollisions();
//     }
//     // std::cout << "interp collisions: " << nCollisions << std::endl;
// }



int RobotGrid::getNCollisions(){
    // return number of collisions found
    int nCollide = 0;
    for (auto r : allRobots){
        if (isCollided(r)) {
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
            stepTowardFold(r, ii);
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
    targetList.clear(); // does clear destroy the shared_ptrs?
    // clear all robot target lists
    for (auto r : allRobots){
        r->targetInds.clear();
        r->assignedTargetInd = -1;
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
    }
    // sort target list in order of priority
    std::sort(targetList.begin(), targetList.end(), sortTargList);

    // associate available targets to robots
    int targInd = -1;
    for (auto t : targetList){
        targInd++;
        int robotInd = -1;
        for (auto r : allRobots){
            robotInd++;
            if (r->isValidTarget(t->x, t->y, t->fiberID)){
                r->targetInds.push_back(targInd);  // should be sorted by priority
                t->robotInds.push_back(robotInd);
            }
        }
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
    int robotInd = -1;
    for (auto r : allRobots){
        robotInd++;
        for (auto targetInd : r->targetInds){
            auto targ = targetList[targetInd];
            if (targ->isAssigned()){
                // target has been assigned to other robot
                continue;
            }
            if (r->isValidTarget(targ->x, targ->y, targ->fiberID)){
                targ->assignRobot(robotInd);
                r->assignTarget(targetInd, targ->x, targ->y, targ->fiberID);
                break; // break from target loop
            }
        }
    }
}

void RobotGrid::pairwiseSwap(){
    // look for pairwise swaps that reduce collisions
    int r1ind = -1;
    for (auto r1 : allRobots){
        r1ind++;
        if (isCollided(r1)){
            // use getNCollisions because the swap may
            // decolide the robot, but may introduce a new
            // collision
            double initialCollisions = getNCollisions();
            for (auto r2ind : r1->neighborInds){
                auto r2 = allRobots[r2ind];
                if (canSwapTarget(r1, r2)){
                    swapTargets(r1ind, r2ind);
                    if (initialCollisions <= getNCollisions()){
                        // swap targets back
                        // collision still exists
                        // or is worse!
                        swapTargets(r1ind, r2ind);
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

bool RobotGrid::canSwapTarget(std::shared_ptr<Robot> r1, std::shared_ptr<Robot> r2){
    auto r1targ = targetList[r1->assignedTargetInd];
    auto r2targ = targetList[r2->assignedTargetInd];
    bool r1canReach = r1->isValidTarget(r2targ->x, r2targ->y, r2targ->fiberID);
    bool r2canReach = r2->isValidTarget(r1targ->x, r1targ->y, r1targ->fiberID);
    return r1canReach and r2canReach;
}

void RobotGrid::swapTargets(int r1Ind, int r2Ind){
    // std::shared_ptr<Target> savedTarget = r1->assignedTarget;
    // r1->assignTarget(r2->assignedTarget);
    // r2->assignTarget(savedTarget);
    // // update target->robot assignments
    // r1->assignedTarget->assignRobot(r1);
    // r2->assignedTarget->assignRobot(r2);
    auto r1 = allRobots[r1Ind];
    auto r2 = allRobots[r2Ind];

    auto r1targ = targetList[r1->assignedTargetInd];
    auto r2targ = targetList[r2->assignedTargetInd];
    r1->assignTarget(r2->assignedTargetInd, r2targ->x, r2targ->y, r2targ->fiberID);
    r2->assignTarget(r1->assignedTargetInd, r1targ->x, r1targ->y, r1targ->fiberID);

    r1targ->assignRobot(r2Ind);
    r2targ->assignRobot(r1Ind);

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

    std::vector<std::shared_ptr<Robot>> idleRobos;
    for (auto robot : allRobots){
        if (robot->targetInds.size()==0){
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
        if (targ->robotInds.size()==0){
            badTargs.push_back(targ);
        }
    }
    return badTargs;
}

std::vector<std::shared_ptr<Target>> RobotGrid::assignedTargets(){
    std::vector<std::shared_ptr<Target>> assignedTargs;

    for (auto t : targetList){
        if (t->isAssigned()){
            assignedTargs.push_back(t);
        }
    }

    return assignedTargs;
}

bool RobotGrid::isValidRobotTarget(int robotInd, int targetInd1){
    auto robot = allRobots[robotInd];
    for (auto targetInd2 : robot->targetInds){
        auto target = targetList[targetInd2];
        if (targetInd1 == targetInd2 and !target->isAssigned()){
					return true;
        }
    }
    return false;
}

void RobotGrid::assignRobot2Target(int robotInd, int targetInd1){
    auto robot = allRobots[robotInd];
    bool targetValid = false;
    for (auto targetInd2 : robot->targetInds){
        auto target = targetList[targetInd2];
        if (targetInd1 == targetInd2 and !target->isAssigned()){
            robot->assignTarget(targetInd1, target->x, target->y, target->fiberID);
            target->assignRobot(robotInd);
            targetValid = true;
            break;
        }
    }
    if (!targetValid){
		for (auto targetInd : robot->targetInds){
            auto target = targetList[targetInd];
            if (targetInd1 == targetInd and target->isAssigned()){
                throw std::runtime_error("assignRobot2Target() failure, targetID already assigned");
            }
		}
        throw std::runtime_error("assignRobot2Target() failure, targetID not valid for robot");
    }
}

void RobotGrid::optimizeTargets(){
    // rewrite!!!
}

bool RobotGrid::isCollided(std::shared_ptr<Robot> robot1){
    // so scope really fucked me on this one?
    // lots of returns fixed it.
    double dist2, collideDist2;
    // check collisions with neighboring robots
    for (auto robotInd : robot1->neighborInds){
        auto robot2 = allRobots[robotInd];
        // squared distance
        dist2 = dist3D_Segment_to_Segment(
                robot2->betaCollisionSegment[0], robot2->betaCollisionSegment[1],
                robot1->betaCollisionSegment[0], robot1->betaCollisionSegment[1]
            );

        collideDist2 = (2*betaCollisionRadius+robot2->collisionBuffer)*
                        (2*betaCollisionRadius+robot1->collisionBuffer);
        if (dist2 < collideDist2){
            // std::cout << "dist " << dist2 - collide_dist_squared << std::endl;
            return true;
        }

    }
    // std::cout << "testing collision" << std::endl;

    return robot1->isFiducialCollided();
}

void RobotGrid::decollideRobot(std::shared_ptr<Robot> robot){
    // remove assigned target if present
    // std::cout << "decolliding robot " << robot->id << std::endl;
    robot->assignedTargetInd = -1;
    for (int ii=0; ii<1000; ii++){
        robot->setXYUniform();
        // nDecollide ++;
        if (!isCollided(robot)){
            // std::cout << "decollide successful " << std::endl;
            break;
        }
    }
    // are we still collided?
    if (isCollided(robot)){
        throw std::runtime_error("Unable do decollide robot!!!");
    }
}

void RobotGrid::stepTowardFold(std::shared_ptr<Robot> robot, int stepNum){
    double currAlpha = robot->alpha;
    double currBeta = robot->beta;
    Eigen::Vector2d alphaPathPoint;
    Eigen::Vector2d betaPathPoint;
    Eigen::Vector2d temp;
    alphaPathPoint(0) = stepNum;
    betaPathPoint(0) = stepNum;
    if (currBeta==180 and currAlpha==0){
        // done folding don't move
        alphaPathPoint(1) = currAlpha;
        betaPathPoint(1) = currBeta;
        robot->alphaPath.push_back(alphaPathPoint);
        robot->betaPath.push_back(betaPathPoint);

        temp(0) = stepNum;
        temp(1) = robot->betaCollisionSegment[0](0); // xAlphaEnd
        robot->roughAlphaX.push_back(temp);
        temp(1) = robot->betaCollisionSegment[0](1); // yAlphaEnd
        robot->roughAlphaY.push_back(temp);
        temp(1) = robot->betaCollisionSegment.back()(0); // xBetaEnd
        robot->roughBetaX.push_back(temp);
        temp(1) = robot->betaCollisionSegment.back()(1); // yBetaEnd
        robot->roughBetaY.push_back(temp);

        return;
    }
    // this is for keeping track of last step
    // only updates if robot hasn't reached fold
    robot->lastStepNum = stepNum;
    // begin trying options pick first that works
    for (int ii=0; ii<robot->alphaBetaArr.rows(); ii++){
        double nextAlpha = currAlpha + robot->alphaBetaArr(ii, 0);
        double nextBeta = currBeta + robot->alphaBetaArr(ii, 1);
        if (nextAlpha > 360){
            nextAlpha = 360;
        }
        if (nextAlpha < 0){
            nextAlpha = 0;
        }
        if (nextBeta > 180){
            nextBeta = 180;
        }
        if (nextBeta < 0){
            nextBeta = 0;
        }
        // if next choice results in no move skip it
        // always favor a move
        if (nextBeta==currBeta and nextAlpha==currAlpha){
            continue;
        }
        robot->setAlphaBeta(nextAlpha, nextBeta);
        if (!isCollided(robot)){
            alphaPathPoint(1) = nextAlpha;
            betaPathPoint(1) = nextBeta;
            robot->alphaPath.push_back(alphaPathPoint);
            robot->betaPath.push_back(betaPathPoint);

            // add alpha/beta xy points

            temp(0) = stepNum;
            // std::cout << "beta orientation size: " << betaOrientation.size() << std::endl;
            // std::cout << "beta model size: " << betaModel.size() << std::endl;
            temp(1) = robot->betaCollisionSegment[0](0); // xAlphaEnd
            // std::cout << "step toward fold " << ii << std::endl;

            robot->roughAlphaX.push_back(temp);

            temp(1) = robot->betaCollisionSegment[0](1); // yAlphaEnd
            robot->roughAlphaY.push_back(temp);
            temp(1) = robot->betaCollisionSegment.back()(0); // xBetaEnd
            robot->roughBetaX.push_back(temp);
            temp(1) = robot->betaCollisionSegment.back()(1); // yBetaEnd
            robot->roughBetaY.push_back(temp);

            return;
        }
    }

    // no move options worked,
    // settle for a non-move
    robot->setAlphaBeta(currAlpha, currBeta);
    alphaPathPoint(1) = currAlpha;
    betaPathPoint(1) = currBeta;
    robot->alphaPath.push_back(alphaPathPoint);
    robot->betaPath.push_back(betaPathPoint);

    // add alpha/beta xy points
    // Eigen::Vector2d temp;
    temp(0) = stepNum;
    temp(1) = robot->betaCollisionSegment[0](0); // xAlphaEnd
    robot->roughAlphaX.push_back(temp);
    temp(1) = robot->betaCollisionSegment[0](1); // yAlphaEnd
    robot->roughAlphaY.push_back(temp);
    temp(1) = robot->betaCollisionSegment.back()(0); // xBetaEnd
    robot->roughBetaX.push_back(temp);
    temp(1) = robot->betaCollisionSegment.back()(1); // yBetaEnd
    robot->roughBetaY.push_back(temp);
}

bool RobotGrid::isCollidedInd(int robotInd){
    auto robot = allRobots[robotInd];
    return isCollided(robot);
}

// void RobotGrid::smoothPath(std::shared_ptr<Robot> robot1, double epsilon){
//     // smooth a previously generated path
//     double interpSmoothAlpha, interpSmoothBeta;
//     int npts;
//     Eigen::Vector2d atemp, btemp;
//     RamerDouglasPeucker(robot1->alphaPath, epsilon, robot1->smoothAlphaPath);
//     // bias alpha positive direction because we are approaching zero
//     npts = robot1->smoothAlphaPath.size();
//     for (int ii=1; ii<npts-1; ii++){
//         // only shift internal (not end) points
//         robot1->smoothAlphaPath[ii](1) = robot1->smoothAlphaPath[ii](1);// + epsilon;
//     }

//     RamerDouglasPeucker(robot1->betaPath, epsilon, robot1->smoothBetaPath);
//     // bias beta negative direction because we are approaching 180
//     // linearly interpolate smooth paths to same step values
//     // as computed
//     // bias alpha positive direction because we are approaching zero
//     npts = robot1->smoothBetaPath.size();
//     for (int ii=1; ii<npts-1; ii++){
//         // only shift internal (not end) points
//         robot1->smoothBetaPath[ii](1) = robot1->smoothBetaPath[ii](1);// - epsilon;
//     }

//     // calculate smoothed alpha betas at every step
//     int nDensePoints = robot1->alphaPath.size();
//     for (int ii=0; ii<nDensePoints; ii++){
//         double xVal = alphaPath[ii](0);
//         atemp(0) = xVal; // interpolation step
//         btemp(0) = xVal;
//         interpSmoothAlpha = linearInterpolate(robot1->smoothAlphaPath, xVal);
//         // bias alpha in positive direction because we're approaching zero
//         atemp(1) = interpSmoothAlpha;
//         interpSmoothAlphaPath.push_back(atemp);
//         interpSmoothBeta = linearInterpolate(robot1->smoothBetaPath, xVal);
//         btemp(1) = interpSmoothBeta;
//         interpSmoothBetaPath.push_back(btemp);

//         // populate interpXY points for alpha/beta ends
//         robot1->setAlphaBeta(interpSmoothAlpha, interpSmoothBeta);
//         atemp(1) = betaCollisionSegment[0](0); // xAlphaEnd
//         interpAlphaX.push_back(atemp);
//         atemp(1) = betaCollisionSegment[0](1); // yAlphaEnd
//         interpAlphaY.push_back(atemp);
//         atemp(1) = betaCollisionSegment.back()(0); // xBetaEnd
//         interpBetaX.push_back(atemp);
//         atemp(1) = betaCollisionSegment.back()(1); // yBetaEnd
//         interpBetaY.push_back(atemp);

//         atemp(1) = 0; // not collided
//         if (isCollided()){
//             atemp(1) = 1;
//         }
//         interpCollisions.push_back(atemp);

//     }

// }

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
