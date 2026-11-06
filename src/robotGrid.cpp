#include <iostream>
#include <stdio.h>      /* printf, scanf, puts, NULL */
// #include <Eigen/Dense>
#include <algorithm>    // std::random_shuffle
#include <chrono>       // std::chrono::system_clock
#include <numeric>
#include "utils.h"
#include "robotGrid.h"

// define constants

// get rid of this hardcoded stuff
const double alphaLenRough = 7.4;
const double betaLenRough = 15;
// certain robots have a restricted range of
// posible alpha beta values due to fiducials.
// path planning will never be able to route a robot
// to some positions even if it is non-fiducial colliding.
// for these robots (with a fiducial neighbor down and right)
// beta < fwC0 + alpha*fwC1 is not ever allowed
// and considered collided (even if it technically isn't)
const double fwC0 = 41.2853;
const double fwC1 = -1.3085;

RobotGrid::RobotGrid(double angStep, double epsilon, int seed)
    : angStep(angStep), epsilon(epsilon), seed(seed)
{
    srand(seed);

    smoothCollisions = 0;
    maxPathSteps = (int)(ceil(1000.0/angStep));
    maxDisplacement = 2*sin(angStep*M_PI/180)*(alphaLenRough+betaLenRough);

    // construct the perturbation list
    for (int ii=-1; ii<2; ii++){
        for (int jj=-1; jj<2; jj++){
            // if (ii == jj == 0){
            //     continue;
            // }
            perturbArray.push_back({ii*angStep, jj*angStep});
        }
    }
}

void RobotGrid::addRobot(
    int robotID, std::string holeID, vec3 basePos, vec3 iHat, vec3 jHat,
    vec3 kHat, vec3 dxyz, double alphaLen, double alphaOffDeg,
    double betaOffDeg, double elementHeight, double scaleFac, vec2 metBetaXY,
    vec2 bossBetaXY, vec2 apBetaXY,
    std::array<vec2, 2> collisionSegBetaXY,
    bool hasApogee, double collisionBuffer, bool lefthanded
){
    // ensure robot id doesn't already exist
    if (initialized){
        throw std::runtime_error("RobotGrid is already initialized, no more robots allowed");
    }
    if (robotDict.count(robotID) > 0){
        throw std::runtime_error("Robot ID already exists");
    }
    robotDict[robotID] = std::make_shared<Robot>(
        robotID, holeID, basePos, iHat, jHat,
        kHat, dxyz, alphaLen, alphaOffDeg,
        betaOffDeg, elementHeight, scaleFac, metBetaXY,
        bossBetaXY, apBetaXY,
        collisionSegBetaXY, angStep,
        hasApogee, collisionBuffer, lefthanded
    );
    // robotDict[robotID]->setCollisionBuffer(collisionBuffer);
}

void RobotGrid::addTarget(long targetID, vec3 xyzWok, FiberType fiberType, double priority){
    if (!initialized){
        throw std::runtime_error("Initialize RobotGrid before adding targets");
    }
    if (targetDict.count(targetID) > 0){
        throw std::runtime_error("Target ID already exists");
    }

    targetDict[targetID] = std::make_shared<Target>(targetID, xyzWok, fiberType, priority);
    // add target to robots and robots to target
    for (auto rPair : robotDict){
        auto r = rPair.second;
        if (isValidAssignment(r->id, targetID)){
            r->validTargetIDs.push_back(targetID);
            targetDict[targetID]->validRobotIDs.push_back(r->id);
        }
    }
}

void RobotGrid::addFiducial(int fiducialID, vec3 xyzWok, double collisionBuffer){
    if (initialized){
        throw std::runtime_error("RobotGrid is already initialized, no more fiducials allowed");
    }
    if (fiducialDict.count(fiducialID) > 0){
        throw std::runtime_error("Fiducial ID already exists");
    }
    fiducialDict[fiducialID] = std::make_shared<Fiducial>(fiducialID, xyzWok, collisionBuffer);
}

void RobotGrid::addGFA(int gfaID, std::array<vec3, 2> collisionSegWokXYZ, double collisionBuffer){
    if (initialized){
        throw std::runtime_error("RobotGrid is already initialized, no more GFAs allowed");
    }
    if (gfaDict.count(gfaID) > 0){
        throw std::runtime_error("GFA ID already exists");
    }
    gfaDict[gfaID] = std::make_shared<GFA>(gfaID, collisionSegWokXYZ, collisionBuffer);
}

void RobotGrid::initGrid(){
    // associate neighbors with fiducials and robots in grid
    double dx, dy, dist, minSep;
    if (initialized){
        throw std::runtime_error("RobotGrid is already initialized, don't do it twice!");
    }
    initialized = true;
    nRobots = robotDict.size();

    for (auto rPair1 : robotDict){
        // add fiducials (potential to collide with)
        auto r1 = rPair1.second;
        for (auto fPair : fiducialDict){
            auto fiducial = fPair.second;
            dx = r1->xPos - fiducial->x;
            dy = r1->yPos - fiducial->y;
            dist = hypot(dx, dy);
            minSep = r1->alphaLen + r1->betaLen + r1->collisionBuffer + fiducial->collisionBuffer;
            if (dist < minSep) {
                r1->addFiducialNeighbor(fiducial->id);
            }
        }
        // add GFAs (potential to collide with)
        for (auto gfaPair : gfaDict){
            auto gfa = gfaPair.second;
            vec3 ppoint = {r1->xPos, r1->yPos, r1->elementHeight};
            auto dist2 = dist3D_Point_to_Segment(
                ppoint,
                gfa->collisionSegWokXYZ[0],
                gfa->collisionSegWokXYZ[1]
            );
            auto dist = sqrt(dist2);
            minSep = r1->alphaLen + r1->betaLen + r1->collisionBuffer + gfa->collisionBuffer;
            // std::cout << "dist2 " << dist2 << " dist " << dist << " minSep " << minSep <<std::endl;
            if (dist < minSep) {
                // std::cout << "adding gfa " << gfa->id << " as neighbor to robot " << r1->id << std::endl;
                r1->addGFANeighbor(gfa->id);
            }
        }
        // initialize all to alpha beta 0,0
        r1->setAlphaBeta(0,0);
        // int r2Ind = -1;
        for (auto rPair2 : robotDict){
            auto r2 = rPair2.second;
            // r2Ind++;
            // add neighbors (potential to collide with)
            if (r1->id==r2->id){
                continue;
            }
            dx = r1->xPos - r2->xPos;
            dy = r1->yPos - r2->yPos;
            dist = hypot(dx, dy);
            minSep = r1->alphaLen + r1->betaLen + r1->collisionBuffer +
                     r2->alphaLen + r2->betaLen + r2->collisionBuffer;
            if (dist < minSep){
                // these robots are neighbors
                r1->addRobotNeighbor(r2->id);
            }
        }
    }
}

std::shared_ptr<Robot> RobotGrid::getRobot(int robotID){
    return robotDict[robotID];
}

void RobotGrid::setCollisionBuffer(double newBuffer){
    for (auto rPair : robotDict){
        auto r = rPair.second;
        r->setCollisionBuffer(newBuffer);
    }
}

void RobotGrid::shrinkCollisionBuffer(double absShrink){
    for (auto rPair : robotDict){
        auto r = rPair.second;
        double oldBuffer = r->collisionBuffer;
        r->setCollisionBuffer(oldBuffer - absShrink);
    }
}

void RobotGrid::growCollisionBuffer(double absGrow){
    for (auto rPair : robotDict){
        auto r = rPair.second;
        double oldBuffer = r->collisionBuffer;
        r->setCollisionBuffer(oldBuffer + absGrow);
    }
}

void RobotGrid::decollideGrid(){
    if (!initialized){
        throw std::runtime_error("Initialize RobotGrid before decollideGrid");
    }
    for (int ii=0; ii<1000; ii++){
        // std::cout << "n collisions " << getNCollisions() << std::endl;
        if (!getNCollisions()){
            break;
        }

        for (auto rPair : robotDict){
            auto robotID = rPair.first;
            // if robot is offline, don't try to decollide
            if (rPair.second->isOffline){
                continue;
            }
            if (isCollided(robotID)){
                decollideRobot(robotID);
            }
        }
    }

    if (getNCollisions()){
        throw std::runtime_error("Unable to decollide this pathologic grid!!!");
    }

}


void RobotGrid::smoothPaths(int points){
    for (auto rPair : robotDict){
        auto r = rPair.second;
        r->smoothVelocity(points);
    }
}


void RobotGrid::simplifyPaths(){
    for (auto rPair : robotDict){
        auto r = rPair.second;
        r->simplifyPath(epsilon);
    }
}

void RobotGrid::verifySmoothed(int totalSteps){
    smoothCollisions = 0;
    for (int ii = 0; ii < totalSteps; ii++){
        for (auto rPair : robotDict){
            auto r = rPair.second;
            r->setAlphaBeta(r->interpSimplifiedAlphaPath[ii][1], r->interpSimplifiedBetaPath[ii][1]);
            // std::cout << " robot id " << r.id << std::endl;
        }
        for (auto rPair : robotDict){
            auto r = rPair.second;
            if (isCollided(r->id)){
                smoothCollidedRobots.insert(r->id);
            }
            // std::cout << " robot id " << r.id << std::endl;
        }
        smoothCollisions += getNCollisions();
    }
}



int RobotGrid::getNCollisions(){
    // return number of collisions found
    int nCollide = 0;
    for (auto rPair : robotDict){
        auto rId = rPair.first;
        if (isCollided(rId)) {
            nCollide++;
        }
    }
    return nCollide;
}

void RobotGrid::clearPaths(){
    for (auto rPair : robotDict){
        auto r = rPair.second;
        // clear any existing path
        r->alphaPath.clear();
        r->betaPath.clear();
        r->simplifiedAlphaPath.clear();
        r->simplifiedBetaPath.clear(); // sparse
        r->interpSimplifiedAlphaPath.clear();
        r->interpSimplifiedBetaPath.clear(); // dense
        r->smoothedAlphaPath.clear();
        r->smoothedBetaPath.clear();
        r->smoothAlphaVel.clear();
        r->smoothBetaVel.clear();
        r->interpAlphaX.clear();
        r->interpAlphaY.clear();
        r->interpBetaX.clear();
        r->interpBetaY.clear(); // smoothed
        r->roughAlphaX.clear();
        r->roughAlphaY.clear();
        r->roughBetaX.clear();
        r->roughBetaY.clear();
        r->scoreVec.clear();


        // r->alphaPath.reserve(maxPathSteps);
        // r->betaPath.reserve(maxPathSteps);
        // r->simplifiedAlphaPath.reserve(maxPathSteps);
        // r->simplifiedBetaPath.reserve(maxPathSteps); // sparse
        // r->interpSimplifiedAlphaPath.reserve(maxPathSteps);
        // r->interpSimplifiedBetaPath.reserve(maxPathSteps); // dense
        // r->smoothedAlphaPath.reserve(maxPathSteps);
        // r->smoothedBetaPath.reserve(maxPathSteps);
        // r->smoothAlphaVel.reserve(maxPathSteps);
        // r->smoothBetaVel.reserve(maxPathSteps);
        // r->interpAlphaX.reserve(maxPathSteps);
        // r->interpAlphaY.reserve(maxPathSteps);
        // r->interpBetaX.reserve(maxPathSteps);
        // r->interpBetaY.reserve(maxPathSteps); // smoothed
        // r->roughAlphaX.reserve(maxPathSteps);
        // r->roughAlphaY.reserve(maxPathSteps);
        // r->roughBetaX.reserve(maxPathSteps);
        // r->roughBetaY.reserve(maxPathSteps);
        // r->scoreVec.reserve(maxPathSteps);

        // r->onTargetVec.clear();
    }

}

void RobotGrid::pathGenMDP(double setGreed, double setPhobia, bool ignoreInitialCollisions){
    // path gen 2 steps towards alpha beta target
    // move greed and phobia to constructor?
    if (!initialized){
        throw std::runtime_error("Initialize RobotGrid before pathGen");
    }

    if (getNCollisions() > 0 && !ignoreInitialCollisions){
        throw std::runtime_error("Grid is Kaiju-collided, cannot generate paths");
    }
    if (setGreed < 0 || setGreed > 1){
        throw std::runtime_error("Greed must be between 0 and 1");
    }
    if (setPhobia < 0 || setPhobia > 1){
        throw std::runtime_error("Phobia must be between 0 and 1");
    }
    for (auto rPair : robotDict){
        auto r = rPair.second;
        // verify that a target alpha beta has been set
        if (!r->hasDestinationAlphaBeta){
            throw std::runtime_error("One or more robots have not received target alpha/beta");
        }
    }

    // greed = setGreed;
    // phobia = setPhobia;
    algType = MDP;
    clearPaths();
    didFail = true;
    int ii;
    std::vector<int> robotIDs;
    for (auto rPair : robotDict){
        robotIDs.push_back(rPair.first);
        rPair.second->setGreedPhobia(setGreed, setPhobia);
    }



    for (ii=0; ii<maxPathSteps; ii++){
        std::random_shuffle(robotIDs.begin(), robotIDs.end());
        bool allAtTarget = true;
        for (auto robotID : robotIDs){
            auto r = robotDict[robotID];

            stepMDP(r, ii);
            r->scoreVec.push_back(r->score());
            if (r->score()!=0) {
                // could just check the last elemet in onTargetVec? same thing.
                // or use robot->score
                allAtTarget = false;
            }
        }

        if (allAtTarget){
            didFail = false;
            break;
        }
    }

    nSteps = ii+1;
}

void RobotGrid::pathGenGreedy(bool stopIfDeadlock, bool ignoreInitialCollisions){
    // path gen greedy steps towards alpha beta target
    if (!initialized){
        throw std::runtime_error("Initialize RobotGrid before pathGen");
    }
    if (getNCollisions() > 0 && !ignoreInitialCollisions){
        throw std::runtime_error("Grid is Kaiju-collided, cannot generate paths");
    }

    for (auto rPair : robotDict){
        auto r = rPair.second;
        // verify that a target alpha beta has been set
        if (!r->hasDestinationAlphaBeta){
            throw std::runtime_error("One or more robots have not received target alpha/beta");
        }
    }
    clearPaths();
    didFail = true;
    // greed = 1;
    // phobia = 0;
    algType = Greedy;
    int ii;



    for (ii=0; ii<maxPathSteps; ii++){

        bool keepGoing;
        bool allAtTarget = true;
        if (stopIfDeadlock){
            keepGoing = false;
        }
        else {
            keepGoing = true;
        }

        for (auto rPair : robotDict){
            auto r = rPair.second;

            stepGreedy(r, ii);
            double robotScore = r->score();
            r->scoreVec.push_back(robotScore);
            if (robotScore!=0) {

                allAtTarget = false;

                // inspect the score vec, if it is changing
                // if it's value has decreased over the last
                // 10 steps, keep going
                if (stopIfDeadlock){
                    if (ii < 12){
                        keepGoing = true;
                    }
                    else {
                        double historicScore = r->scoreVec[ii-10];
                        if (robotScore/historicScore < 0.99){
                            keepGoing = true;
                        }
                    }
                }
            }

        }

        if (allAtTarget){
            // std::cout << "all at target" << std::endl;
            didFail = false;
            break;
        }

        if (!keepGoing){
            // std::cout << "stopped due to deadlock" << std::endl;
            break;
        }

        // next check whether or not we're deadlocked

    }

    nSteps = ii+1;
}

void RobotGrid::pathGenExplode(double deg2move){

    // path gen 2 steps towards alpha beta target
    if (!initialized){
        throw std::runtime_error("Initialize RobotGrid before pathGen");
    }

    clearPaths();
    int ii;
    double steps2move = deg2move / angStep;
    for (ii=0; ii<steps2move; ii++){

        for (auto rPair : robotDict){
            auto r = rPair.second;

            stepDecollide(r, ii);
        }


    }
    // hack in destinations for all robots
    for (auto rPair : robotDict){
        auto r = rPair.second;
        r->setDestinationAlphaBeta(r->alpha, r->beta);
    }

    nSteps = ii;
    didFail = false;
}


void RobotGrid::pathGenExplodeOne(double deg2move, int robotID){

    // path gen 2 steps towards alpha beta target
    if (!initialized){
        throw std::runtime_error("Initialize RobotGrid before pathGen");
    }

    // hack in destinations for all robots
    // so robots will not move (except for the desired robot)
    for (auto rPair : robotDict){
        auto r = rPair.second;
        r->setDestinationAlphaBeta(r->alpha, r->beta);
    }

    clearPaths();
    int ii;
    double steps2move = deg2move / angStep;
    for (ii=0; ii<steps2move; ii++){

        for (auto rPair : robotDict){
            auto r = rPair.second;
            if (rPair.first == robotID){
                stepDecollide(r, ii);
            }
            else {
                // doesn't actually move because destinations
                // have been set to current positions
                // this just fills out the arrays
                stepGreedy(r, ii);
            }
        }
    }

    // hack in destination for single robot that moved

    auto r = robotDict.at(robotID);
    // auto rAlpha = r->alpha;
    // auto rBeta = r->beta;
    r->setDestinationAlphaBeta(r->alpha, r->beta);
    nSteps = ii;
    didFail = false;

}


void RobotGrid::clearTargetDict(){
    targetDict.clear(); // does clear destroy the shared_ptrs?
    // clear all robot target lists
    for (auto rPair : robotDict){
        auto r = rPair.second;
        r->validTargetIDs.clear();
        r->clearAssignment();
    }
}


std::vector<int> RobotGrid::unassignedRobots(){
    // return the ids of unassigned robots
    std::vector<int> idleRobotIDs;
    for (auto rPair : robotDict){
        auto robot = rPair.second;
        if (!robot->isAssigned()){
            idleRobotIDs.push_back(robot->id);
        }
    }
    return idleRobotIDs;
}

std::vector<int> RobotGrid::targetlessRobots(){
    // return the ids of robots that can reach no targets in targetDict
    std::vector<int> idleRobotIDs;
    for (auto rPair : robotDict){
        auto robot = rPair.second;
        if (robot->validTargetIDs.size()==0){
            idleRobotIDs.push_back(robot->id);
        }
    }
    return idleRobotIDs;
}

std::vector<long> RobotGrid::unreachableTargets(){
    // return list of targetIDs that are not reachable
    // by any robot
    if (targetDict.size()==0){
        throw std::runtime_error("unreachableTargets() failure, target list not yet set");
    }
    std::vector<long> badTargs;
    for (auto tPair : targetDict){
        auto targ = tPair.second;
        if (targ->validRobotIDs.size()==0){
            badTargs.push_back(targ->id);
        }
    }
    return badTargs;
}

std::vector<long> RobotGrid::assignedTargets(){
    // return list of targetIDs that have been assigned
    // to robots
    std::vector<long> assignedTargIDs;

    for (auto tPair : targetDict){
        auto t = tPair.second;
        if (t->isAssigned()){
            assignedTargIDs.push_back(t->id);
        }
    }

    return assignedTargIDs;
}



void RobotGrid::unassignTarget(long targID){
    // clear the the target assignment, and the
    // robot to which it's assigned
    auto target = targetDict[targID];
    if (!target->isAssigned()){
        // do nothing, target isnt assigned
        return;
    }
    auto robot = robotDict[target->assignedRobotID];
    target->clearAssignment();
    robot->clearAssignment();
}

void RobotGrid::unassignRobot(int robotID){
    // clear the the target assignment, and the
    // robot to which it's assigned
    auto robot = robotDict[robotID];
    if (!robot->isAssigned()){
        // do nothing, target isnt assigned
        return;
    }
    auto target = targetDict[robot->assignedTargetID];
    target->clearAssignment();
    robot->clearAssignment();
}

void RobotGrid::assignRobot2Target(int robotID, long targetID){
    // releases robot's previous target if present
    // releases target's previous robot if present
    auto robot = robotDict[robotID];
    auto target = targetDict[targetID];

    int ii = std::count(robot->validTargetIDs.begin(), robot->validTargetIDs.end(), targetID);
    if (ii == 0){
        throw std::runtime_error("target not valid for robot");
    }
    unassignRobot(robotID);
    unassignTarget(targetID);
    target->assignRobot(robotID);
    robot->assignTarget(targetID);
    auto ab = robot->alphaBetaFromWokXYZ(target->xyzWok, target->fiberType);
    robot->setAlphaBeta(ab[0], ab[1]);
}

bool RobotGrid::isValidAssignment(int robotID, long targetID){
    auto robot = robotDict[robotID];
    auto target = targetDict[targetID];

    if (target->fiberType == ApogeeFiber and !robot->hasApogee){
        return false;
    }


    auto ab = robot->alphaBetaFromWokXYZ(target->xyzWok, target->fiberType);
    // check alpha beta valid
    if (std::isnan(ab[0]) or std::isnan(ab[1])){
        return false;
    }

    // save current alpha beta
    double savedAlpha = robot->alpha;
    double savedBeta = robot->beta;
    robot->setAlphaBeta(ab[0], ab[1]);
    bool returnValue = true;

    auto collidedFiducials = fiducialColliders(robotID);
    if (collidedFiducials.size() != 0){
        // this target interferes with a fiducial which is immobile
        robot->setAlphaBeta(savedAlpha, savedBeta);
        returnValue = false;
    }
    auto collidedGFAs = gfaColliders(robotID);
    if (collidedGFAs.size() != 0){
        // this target interferes with a GFA which is immobile
        robot->setAlphaBeta(savedAlpha, savedBeta);
        returnValue = false;
    }
    // reset alpha beta
    robot->setAlphaBeta(savedAlpha, savedBeta);
    return returnValue;
}



bool RobotGrid::isCollided(int robotID){
    auto robotsColliding = robotColliders(robotID);
    if (robotsColliding.size() != 0){
        return true;
    }
    auto fiducialsColliding = fiducialColliders(robotID);
    if (fiducialsColliding.size() != 0){
        return true;
    }
    auto gfasColliding = gfaColliders(robotID);
    if (gfasColliding.size() != 0){
        return true;
    }
    return false;
}

std::tuple<bool, bool, bool, std::vector<int>> RobotGrid::isCollidedWithAssigned(int robotID){
    bool collided, fiducial_collided, gfa_collided;
    std::vector<int> assignedRobotsColliding;
    auto robotsColliding = robotColliders(robotID);
    if (robotsColliding.size() != 0){
        for (auto robotColliding : robotsColliding) {
            if(robotDict[robotColliding]->isAssigned()) {
	           assignedRobotsColliding.push_back(robotColliding);
            }
        }
    }
    auto fiducialsColliding = fiducialColliders(robotID);
    auto gfasColliding = gfaColliders(robotID);

    fiducial_collided = (fiducialsColliding.size() != 0);
    gfa_collided = (gfasColliding.size() != 0);

    collided = (fiducial_collided || gfa_collided || (assignedRobotsColliding.size() != 0));


    return std::make_tuple(collided, fiducial_collided, gfa_collided, assignedRobotsColliding);
}

void RobotGrid::homeRobot(int robotID){
    unassignRobot(robotID);
    auto robot = robotDict[robotID];
		robot->setAlphaBeta(0., 180.);
}

std::tuple<bool, bool, bool, std::vector<int>> RobotGrid::wouldCollideWithAssigned(int robotID, long targID){
    long currentTargetID;
    int currentRobotID;
    std::tuple<bool, bool, bool, std::vector<int>> result;

    auto robot = robotDict[robotID];
    if(robot->isAssigned())
    currentTargetID = robot->assignedTargetID;
    else
    currentTargetID = -1;

    currentRobotID = targetDict[targID]->assignedRobotID;

    assignRobot2Target(robotID, targID);
    result = isCollidedWithAssigned(robotID);

    if(currentTargetID >= 0)
    assignRobot2Target(robotID, currentTargetID);
    else
    unassignRobot(robotID);

    if(currentRobotID >= 0)
    assignRobot2Target(currentRobotID, targID);

    return result;
    }


bool RobotGrid::neighborEncroachment(std::shared_ptr<Robot> robot1, double encroachDist = 3){
    // score, separation2
    // look ahead and see robots getting close
    double dist2;

    // turn off neighbor encroachment if robot is disables
    if (robot1->isOffline){
        return false;
    }

    // check collisions with neighboring robots
    for (auto otherRobotID : robot1->robotNeighbors){
        auto robot2 = robotDict[otherRobotID];
        // squared distance returned
        double minDist = robot1->collisionBuffer + robot2->collisionBuffer + encroachDist*maxDisplacement;
        dist2 = dist3D_Segment_to_Segment(
                robot2->collisionSegWokXYZ[0], robot2->collisionSegWokXYZ[1],
                robot1->collisionSegWokXYZ[0], robot1->collisionSegWokXYZ[1]
            );
        if (dist2 < (minDist*minDist)){

            return true;
        }
    }
    return false;
}


std::vector<int> RobotGrid::robotColliders(int robotID){

    std::vector<int> collidingNeighbors;
    double dist2, collideDist2, dist;
    // check collisions with neighboring robots
    auto robot1 = robotDict[robotID];

    for (auto otherRobotID : robot1->robotNeighbors){
        auto robot2 = robotDict[otherRobotID];
        // squared distance returned
        dist2 = dist3D_Segment_to_Segment(
                robot2->collisionSegWokXYZ[0], robot2->collisionSegWokXYZ[1],
                robot1->collisionSegWokXYZ[0], robot1->collisionSegWokXYZ[1]
            );
        collideDist2 = (robot1->collisionBuffer + robot2->collisionBuffer + maxDisplacement)
                       * (robot1->collisionBuffer + robot2->collisionBuffer + maxDisplacement);
        // dist = sqrt(dist2);
        // collideDist2 = (2*collisionBuffer)*(2*collisionBuffer);
        if (dist2 < collideDist2){
            // std::cout << "dist " << dist2 - collide_dist_squared << std::endl;
            collidingNeighbors.push_back(robot2->id);
        }

    }
    return collidingNeighbors;
}

std::vector<int> RobotGrid::fiducialColliders(int robotID){

    std::vector<int> collidingNeighbors;
    auto robot = robotDict[robotID];
    double dist2, collideDist2;

    for (auto fiducialID : robot->fiducialNeighbors){
        auto fiducial = fiducialDict[fiducialID];
        // squared distance
        dist2 = dist3D_Point_to_Segment(
                fiducial->xyzWok, robot->collisionSegWokXYZ[0],
                robot->collisionSegWokXYZ[1]
                );
        collideDist2 =  (robot->collisionBuffer+fiducial->collisionBuffer) *
                        (robot->collisionBuffer+fiducial->collisionBuffer);

        if (dist2 < collideDist2){
            collidingNeighbors.push_back(fiducial->id);
        }
    }

    // check for special robots with a fiducial neighbor
    // (tagged during grid construction) located down and right
    // for these robots some small range of alpha/beta values are
    // not allowed
    if (robot->fiducialWatch){
        if (robot->beta < fwC0 + robot->alpha*fwC1){
            collidingNeighbors.push_back(-1);
        }
    }

    return collidingNeighbors;

}

std::vector<int> RobotGrid::gfaColliders(int robotID){

    std::vector<int> collidingNeighbors;
    auto robot = robotDict[robotID];
    double dist2, collideDist2;

    for (auto gfaID : robot->gfaNeighbors){
        auto gfa = gfaDict[gfaID];
        dist2 = dist3D_Segment_to_Segment(
                gfa->collisionSegWokXYZ[0], gfa->collisionSegWokXYZ[1],
                robot->collisionSegWokXYZ[0], robot->collisionSegWokXYZ[1]
            );

        collideDist2 =  (robot->collisionBuffer+gfa->collisionBuffer) *
                        (robot->collisionBuffer+gfa->collisionBuffer);

        if (dist2 < collideDist2){
            collidingNeighbors.push_back(gfa->id);
        }
    }
    return collidingNeighbors;

}

std::vector<int> RobotGrid::getCollidedRobotRobotList(){
    std::vector<int> robotIDList;
    for (auto rPair : robotDict){
        auto robotID = rPair.first;
        auto colliders = robotColliders(robotID);
        if (!colliders.empty()){
            robotIDList.push_back(robotID);
        }
    }
    return robotIDList;
}


std::vector<int> RobotGrid::getCollidedRobotFiducialList(){
    std::vector<int> robotIDList;
    for (auto rPair : robotDict){
        auto robotID = rPair.first;
        auto colliders = fiducialColliders(robotID);
        if (!colliders.empty()){
            robotIDList.push_back(robotID);
        }
    }
    return robotIDList;

}

std::vector<int> RobotGrid::getCollidedRobotGFAList(){
    std::vector<int> robotIDList;
    for (auto rPair : robotDict){
        auto robotID = rPair.first;
        auto colliders = gfaColliders(robotID);
        if (!colliders.empty()){
            robotIDList.push_back(robotID);
        }
    }
    return robotIDList;
}


std::vector<int> RobotGrid::getCollidedRobotList(){
    std::vector<int> robotIDList;
    for (auto rPair : robotDict){
        auto robotID = rPair.first;
        if (isCollided(robotID)){
            robotIDList.push_back(robotID);
        }
    }
    return robotIDList;
}


bool RobotGrid::throwAway(int robotID){
    // return true if worked
    // robot gets new alpha beta position
    // if return is false robot is unchanged
    auto robot = robotDict[robotID];
    auto currAlpha = robot->alpha;
    auto currBeta = robot->beta;
    // attempt to keep beta as small as possible
    // loop over it first
    for (int ii=0; ii<100000; ii++){
        robot->setXYUniform();
        if (!isCollided(robotID)){
            return true;
        }
    }
    // couldn't find replacement!
    robot->setAlphaBeta(currAlpha, currBeta);
    return false;
}


void RobotGrid::decollideRobot(int robotID){
    // warning, if decollide doesn't work
    // no error is raised!!!
    // remove assigned target if present
    unassignRobot(robotID);
    auto robot = robotDict[robotID];
    for (int ii=0; ii<1000; ii++){
        robot->setXYUniform();
        // nDecollide ++;
        if (!isCollided(robotID)){
            // std::cout << "decollide successful " << std::endl;
            break;
        }
    }

    // may want to fail hard if we still collided?
    // if (isCollided(robot)){
    //     throw std::runtime_error("Unable do decollide robot!!!");
    // }
}

std::vector<int> RobotGrid::deadlockedRobots(){
    std::vector<int> deadlockedRobotIDs;


    for (auto rPair : robotDict){
        auto robot = rPair.second;
        if (algType == Fold){
            if (robot->alpha != 0 or robot->beta != 180){
                deadlockedRobotIDs.push_back(robot->id);
            }
        }
        else {
            if (robot->score() != 0){
                deadlockedRobotIDs.push_back(robot->id);
            }
            else if (neighborEncroachment(robot)){
                deadlockedRobotIDs.push_back(robot->id);
            }
        }
    }
    return deadlockedRobotIDs;
}


// handle things near limits of robot motion
vec2 handleLimits(double currAlpha, double currBeta, double nextAlpha, double nextBeta){
    vec2 nextAlphaBeta;

    // handle alpha lims
    // if robots currently beyond limits
    // don't allow them to go further beyond limits
    if (currAlpha < 0 && nextAlpha < currAlpha){
        nextAlpha = currAlpha; // don't move further the wrong direction
    }

    else if (currAlpha > 360 && nextAlpha > currAlpha){
        nextAlpha = currAlpha;
    }
    // robots currently in bounds
    else if (nextAlpha > 360){
        nextAlpha = 360;
    }
    else if (nextAlpha < 0){
        nextAlpha = 0;
    }

    if (currBeta < 0 && nextBeta < currBeta){
        nextBeta = currBeta; // don't move further the wrong direction
    }

    else if (currBeta > 360 && nextBeta > currBeta){
        nextBeta = currBeta;
    }
    // robots currently in bounds
    else if (nextBeta > 360){
        nextBeta = 360;
    }
    else if (nextBeta < 0){
        nextBeta = 0;
    }

    nextAlphaBeta[0] = nextAlpha;
    nextAlphaBeta[1] = nextBeta;
    return nextAlphaBeta;

    // if robots moving beyond limits, don't let them do it



}

void RobotGrid::stepDecollide(std::shared_ptr<Robot> robot, int stepNum){
    double currAlpha = robot->alpha;
    double currBeta = robot->beta;
    vec2 alphaPathPoint;
    vec2 betaPathPoint;
    vec2 temp;
    alphaPathPoint[0] = stepNum;
    betaPathPoint[0] = stepNum;


    double maxDist = -1e9; // to be maximized
    double bestAlpha = 0;  // solved in loop
    double bestBeta = 0;
    // this is for keeping track of last step
    // only updates if robot hasn't reached fold
    robot->lastStepNum = stepNum;


    for (auto dAlphaBeta : perturbArray){
        double nextAlpha = currAlpha + dAlphaBeta[0];
        double nextBeta = currBeta + dAlphaBeta[1];

        vec2 nextAlphaBeta = handleLimits(currAlpha, currBeta, nextAlpha, nextBeta);

        nextAlpha = nextAlphaBeta[0];
        nextBeta =nextAlphaBeta[1];


        robot->setAlphaBeta(nextAlpha, nextBeta);

        auto collideDist = minCollideDist(robot->id);
        if (collideDist > maxDist){
            maxDist = collideDist;
            bestAlpha = nextAlpha;
            bestBeta = nextBeta;
        }
    }


    robot->setAlphaBeta(bestAlpha, bestBeta);
    alphaPathPoint[1] = bestAlpha;
    betaPathPoint[1] = bestBeta;
    robot->alphaPath.push_back(alphaPathPoint);
    robot->betaPath.push_back(betaPathPoint);

    // add alpha/beta xy points

    temp[0] = stepNum;
    temp[1] = robot->collisionSegWokXYZ[0][0]; // xAlphaEnd

    robot->roughAlphaX.push_back(temp);

    temp[1] = robot->collisionSegWokXYZ[0][1]; // yAlphaEnd
    robot->roughAlphaY.push_back(temp);
    temp[1] = robot->collisionSegWokXYZ.back()[0]; // xBetaEnd
    robot->roughBetaX.push_back(temp);
    temp[1] = robot->collisionSegWokXYZ.back()[1]; // yBetaEnd
    robot->roughBetaY.push_back(temp);

    return;
}

double RobotGrid::minCollideDist(int robotID){

    double dist2, dist;
    // check collisions with neighboring robots
    double minDist = 1e9;  // to be minimized
    auto robot1 = robotDict[robotID];


    // search through robot neighbors for min dist
    for (auto otherRobotID : robot1->robotNeighbors){
        auto robot2 = robotDict[otherRobotID];
        // squared distance returned
        dist2 = dist3D_Segment_to_Segment(
                robot2->collisionSegWokXYZ[0], robot2->collisionSegWokXYZ[1],
                robot1->collisionSegWokXYZ[0], robot1->collisionSegWokXYZ[1]
            );
        dist = sqrt(dist2) - robot2->collisionBuffer - robot1->collisionBuffer;
        if (dist < minDist){
            minDist = dist;
        }
    }

    for (auto fiducialID : robot1->fiducialNeighbors){
        auto fiducial = fiducialDict[fiducialID];
        // squared distance
        dist2 = dist3D_Point_to_Segment(
                fiducial->xyzWok, robot1->collisionSegWokXYZ[0],
                robot1->collisionSegWokXYZ[1]
                );
        dist = sqrt(dist2) - fiducial->collisionBuffer - robot1->collisionBuffer;
        if (dist < minDist){
            minDist = dist;
        }
    }

    for (auto gfaID : robot1->gfaNeighbors){
        auto gfa = gfaDict[gfaID];
        // squared distance
        dist2 = dist3D_Segment_to_Segment(
                gfa->collisionSegWokXYZ[0], gfa->collisionSegWokXYZ[1],
                robot1->collisionSegWokXYZ[0], robot1->collisionSegWokXYZ[1]
                );
        dist = sqrt(dist2) - gfa->collisionBuffer - robot1->collisionBuffer;
        if (dist < minDist){
            minDist = dist;
        }
    }


    return minDist;
}

void RobotGrid::stepGreedy(std::shared_ptr<Robot> robot, int stepNum){

    double score;
    double currAlpha = robot->alpha;
    double currBeta = robot->beta;
    double bestAlpha, bestBeta, bestScore, nextAlpha, nextBeta;
    bestAlpha = currAlpha;
    bestBeta = currBeta;
    bestScore = 1e16; // to be minimized

    vec2 alphaPathPoint;
    vec2 betaPathPoint;
    vec2 temp;

    alphaPathPoint[0] = stepNum;
    betaPathPoint[0] = stepNum;


    if (stepNum==0 || robot->score()==0){
        // at target or on first step, don't move
        alphaPathPoint[1] = currAlpha;
        betaPathPoint[1] = currBeta;
        robot->alphaPath.push_back(alphaPathPoint);
        robot->betaPath.push_back(betaPathPoint);

        // note make collision segment just two points

        temp[0] = stepNum;
        temp[1] = robot->collisionSegWokXYZ[0][0]; // xAlphaEnd
        robot->roughAlphaX.push_back(temp);
        temp[1] = robot->collisionSegWokXYZ[0][1]; // yAlphaEnd
        robot->roughAlphaY.push_back(temp);
        temp[1] = robot->collisionSegWokXYZ.back()[0]; // xBetaEnd
        robot->roughBetaX.push_back(temp);
        temp[1] = robot->collisionSegWokXYZ.back()[1]; // yBetaEnd
        robot->roughBetaY.push_back(temp);

        return;
    }

    robot->lastStepNum = stepNum;

    std::random_shuffle(perturbArray.begin(), perturbArray.end());
    // check all move combinations for each axis
    for (auto dAlphaBeta : perturbArray){
        nextAlpha = currAlpha + dAlphaBeta[0];
        nextBeta = currBeta + dAlphaBeta[1];
        // careful not to overshoot
        if (currAlpha > robot->destinationAlpha and nextAlpha <= robot->destinationAlpha){
            nextAlpha = robot->destinationAlpha;
        }
        if (currAlpha < robot->destinationAlpha and nextAlpha >= robot->destinationAlpha){
            nextAlpha = robot->destinationAlpha;
        }
        if (currBeta > robot->destinationBeta and nextBeta <= robot->destinationBeta){
            nextBeta = robot->destinationBeta;
        }
        if (currBeta < robot->destinationBeta and nextBeta >= robot->destinationBeta){
            nextBeta = robot->destinationBeta;
        }
        // handle limits of travel
        // can probably ditch this as target
        // must be in range anyways
        vec2 nextAlphaBeta = handleLimits(currAlpha, currBeta, nextAlpha, nextBeta);
        nextAlpha = nextAlphaBeta[0];
        nextBeta =nextAlphaBeta[1];

        robot->setAlphaBeta(nextAlpha, nextBeta);
        score = robot->score();

        if (!isCollided(robot->id)){
            if (score < bestScore){
                bestScore = score;
                bestAlpha = nextAlpha;
                bestBeta = nextBeta;

            }

            else if (score == bestScore and randomSample() >= 0.5){
                // flip a coin to see whether to accept
                bestScore = score;
                bestAlpha = nextAlpha;
                bestBeta = nextBeta;
            }
        }
    } // end loop over perturbations

    // set alpha beta to best found option
    robot->setAlphaBeta(bestAlpha, bestBeta);
    alphaPathPoint[1] = bestAlpha;
    betaPathPoint[1] = bestBeta;
    robot->alphaPath.push_back(alphaPathPoint);
    robot->betaPath.push_back(betaPathPoint);

    temp[0] = stepNum;
    temp[1] = robot->collisionSegWokXYZ[0][0]; // xAlphaEnd
    robot->roughAlphaX.push_back(temp);
    temp[1] = robot->collisionSegWokXYZ[0][1]; // yAlphaEnd
    robot->roughAlphaY.push_back(temp);
    temp[1] = robot->collisionSegWokXYZ.back()[0]; // xBetaEnd
    robot->roughBetaX.push_back(temp);
    temp[1] = robot->collisionSegWokXYZ.back()[1]; // yBetaEnd
    robot->roughBetaY.push_back(temp);


}

void RobotGrid::stepMDP(std::shared_ptr<Robot> robot, int stepNum){

    double score, dist2, localEnergy;
    double nextAlpha, nextBeta;
    bool doPhobia;
    double currAlpha = robot->alpha;
    double currBeta = robot->beta;
    double bestAlpha, bestBeta, bestScore;
    bestAlpha = currAlpha;
    bestBeta = currBeta;
    bestScore = 1e16; // to be minimized

    vec2 alphaPathPoint;
    vec2 betaPathPoint;
    vec2 temp;

    alphaPathPoint[0] = stepNum;
    betaPathPoint[0] = stepNum;


    if (stepNum==0 || (robot->score()==0 && !neighborEncroachment(robot))){
        // either at first step, or done folding no one knocking don't move
        alphaPathPoint[1] = currAlpha;
        betaPathPoint[1] = currBeta;
        robot->alphaPath.push_back(alphaPathPoint);
        robot->betaPath.push_back(betaPathPoint);

        // note make collision segment just two points

        temp[0] = stepNum;
        temp[1] = robot->collisionSegWokXYZ[0][0]; // xAlphaEnd
        robot->roughAlphaX.push_back(temp);
        temp[1] = robot->collisionSegWokXYZ[0][1]; // yAlphaEnd
        robot->roughAlphaY.push_back(temp);
        temp[1] = robot->collisionSegWokXYZ.back()[0]; // xBetaEnd
        robot->roughBetaX.push_back(temp);
        temp[1] = robot->collisionSegWokXYZ.back()[1]; // yBetaEnd
        robot->roughBetaY.push_back(temp);

        return;
    }

    robot->lastStepNum = stepNum;
    // begin looping over all possible moves
    // shuffle move options to ensure they are visited
    // in no particular order
    std::random_shuffle(perturbArray.begin(), perturbArray.end());

    // decide whether we're minimizing phobia
    // or minimizing score

    doPhobia = randomSample() < robot->phobia;


    for (auto dAlphaBeta : perturbArray){
        nextAlpha = currAlpha + dAlphaBeta[0];
        nextBeta = currBeta + dAlphaBeta[1];
        // careful not to overshoot
        if (currAlpha > robot->destinationAlpha and nextAlpha <= robot->destinationAlpha){
            nextAlpha = robot->destinationAlpha;
        }
        if (currAlpha < robot->destinationAlpha and nextAlpha >= robot->destinationAlpha){
            nextAlpha = robot->destinationAlpha;
        }
        if (currBeta > robot->destinationBeta and nextBeta <= robot->destinationBeta){
            nextBeta = robot->destinationBeta;
        }
        if (currBeta < robot->destinationBeta and nextBeta >= robot->destinationBeta){
            nextBeta = robot->destinationBeta;
        }
        // handle limits of travel
        vec2 nextAlphaBeta = handleLimits(currAlpha, currBeta, nextAlpha, nextBeta);
        nextAlpha = nextAlphaBeta[0];
        nextBeta =nextAlphaBeta[1];

        robot->setAlphaBeta(nextAlpha, nextBeta);
        if (isCollided(robot->id)){
            // don't consider this a viable option
            // move to next option
            break;
        }



        // score is min possible steps till goal
        localEnergy = 0;

        // compute robot's local energy, and check for collision
        for (auto otherRobotID : robot->robotNeighbors){
            auto otherRobot = robotDict[otherRobotID];
            dist2 = dist3D_Segment_to_Segment(
                otherRobot->collisionSegWokXYZ[0], otherRobot->collisionSegWokXYZ[1],
                robot->collisionSegWokXYZ[0], robot->collisionSegWokXYZ[1]
            );

            localEnergy += 1/dist2;

            double collideDist2 = (robot->collisionBuffer + otherRobot->collisionBuffer + maxDisplacement) *
                                  (robot->collisionBuffer + otherRobot->collisionBuffer + maxDisplacement);
        }

        if (doPhobia){
            score = localEnergy;
        }
        else {
            score = robot->score();
        }

        if (score < bestScore and randomSample() < robot->greed){
        // almost always pick a better score
            bestScore = score;
            bestAlpha = nextAlpha;
            bestBeta = nextBeta;
        }
        else if (score == bestScore and randomSample() > 0.5){
            // if score is same switch to new
            // state with 0.5 probability
            bestScore = score;
            bestAlpha = nextAlpha;
            bestBeta = nextBeta;
        }
    } // end loop over perturbations

    // set alpha beta to best found option
    robot->setAlphaBeta(bestAlpha, bestBeta);
    alphaPathPoint[1] = bestAlpha;
    betaPathPoint[1] = bestBeta;
    robot->alphaPath.push_back(alphaPathPoint);
    robot->betaPath.push_back(betaPathPoint);

    temp[0] = stepNum;
    temp[1] = robot->collisionSegWokXYZ[0][0]; // xAlphaEnd
    robot->roughAlphaX.push_back(temp);
    temp[1] = robot->collisionSegWokXYZ[0][1]; // yAlphaEnd
    robot->roughAlphaY.push_back(temp);
    temp[1] = robot->collisionSegWokXYZ.back()[0]; // xBetaEnd
    robot->roughBetaX.push_back(temp);
    temp[1] = robot->collisionSegWokXYZ.back()[1]; // yBetaEnd
    robot->roughBetaY.push_back(temp);
}


void RobotGrid::pathGenMDP2(double setGreed, double setPhobia, bool ignoreInitialCollisions, int nTries){
    // path gen 2 steps towards alpha beta target
    // move greed and phobia to constructor?
    int ii, jj;

    if (!initialized){
        throw std::runtime_error("Initialize RobotGrid before pathGen");
    }

    if (getNCollisions() > 0 && !ignoreInitialCollisions){
        throw std::runtime_error("Grid is Kaiju-collided, cannot generate paths");
    }
    if (setGreed < 0 || setGreed > 1){
        throw std::runtime_error("Greed must be between 0 and 1");
    }
    if (setPhobia < 0 || setPhobia > 1){
        throw std::runtime_error("Phobia must be between 0 and 1");
    }
    for (auto rPair : robotDict){
        auto r = rPair.second;
        // verify that a target alpha beta has been set
        if (!r->hasDestinationAlphaBeta){
            throw std::runtime_error("One or more robots have not received target alpha/beta");
        }
    }

    // int nTries = 5;
    // greed = setGreed;
    // phobia = setPhobia;
    algType = MDP;
    std::vector<int> robotIDs;
    for (auto rPair : robotDict){
        robotIDs.push_back(rPair.first);
        rPair.second->setGreedPhobia(setGreed, setPhobia);
        rPair.second->saveAlphaBeta();
    }


    for (jj=0; jj<nTries; jj++){

        std::cout << "on path gen iter " << jj << std::endl;
        clearPaths();
        didFail = true;

        for (auto rPair2 : robotDict){
            rPair2.second->setAlphaBeta(rPair2.second->alphaInit, rPair2.second->betaInit);
        }



        for (ii=0; ii<maxPathSteps; ii++){
            std::random_shuffle(robotIDs.begin(), robotIDs.end());
            bool allAtTarget = true;
            for (auto robotID : robotIDs){
                auto r = robotDict[robotID];
                stepMDP2(r, ii);
                r->scoreVec.push_back(r->score());
                if (r->score()!=0) {
                    // could just check the last elemet in onTargetVec? same thing.
                    // or use robot->score
                    allAtTarget = false;
                }
            }

            if (allAtTarget){
                didFail = false;
                break;
            }
        }

        if (!didFail){
            break;
        }

        // this iteration failed.  find a robot to nudge
        // then try again
        double largestBeta = -999;
        int robot2nudge;
        for (auto robotID : robotIDs){
            auto r = robotDict[robotID];
            if (r->score()!=0 and !r->nudge){
                if (r->beta > largestBeta){
                    largestBeta = r->beta;
                    robot2nudge = r->id;
                }
            }
        }

        if (largestBeta > -5){
            robotDict[robot2nudge]->nudge = true;
            std::cout << "nudging robot " << robot2nudge << std::endl;
        }

        // try another method, nudge all robots that didn't make it home
        // as long as they don't have a nudged neighbor
        // for (auto robotID : robotIDs){
        //     auto r = robotDict[robotID];
        //     if (r->score()!=0){
        //         // just nudge it
        //         r->nudge = true;
        //         std::cout << "nudging robot " << robotID << std::endl;
        //         // if (!r->nudge){
        //         //     // check if neighbors are nudged
        //         //     bool neighborNudged = false;
        //         //     for (auto otherRobotID : r->robotNeighbors){
        //         //         auto otherRobot = robotDict[otherRobotID];
        //         //         if (otherRobot->nudge){
        //         //             neighborNudged = true;
        //         //             break;
        //         //         }
        //         //     }
        //         //     if (!neighborNudged){
        //         //         std::cout << "nudging robot " << robotID << std::endl;
        //         //         r->nudge = true;
        //         //     }
        //         // }
        //     }
        // }
    }

    nSteps = ii+1;
    mdp2iter = jj;

}

void RobotGrid::stepMDP2(std::shared_ptr<Robot> robot, int stepNum){

    double score, dist2, localEnergy;
    double nextAlpha, nextBeta;
    bool doPhobia;
    double currAlpha = robot->alpha;
    double currBeta = robot->beta;
    double bestAlpha, bestBeta, bestScore;
    double greed;
    bool isEncroaching;
    bool atDestination;
    bestAlpha = currAlpha;
    bestBeta = currBeta;
    bestScore = 1e16; // to be minimized

    vec2 alphaPathPoint;
    vec2 betaPathPoint;
    vec2 temp;

    alphaPathPoint[0] = stepNum;
    betaPathPoint[0] = stepNum;

    isEncroaching = neighborEncroachment(robot, 20);
    atDestination = robot->score()==0;


    if (stepNum==0 || (atDestination && !isEncroaching)){
        // either at first step, or done folding no one knocking don't move
        alphaPathPoint[1] = currAlpha;
        betaPathPoint[1] = currBeta;
        robot->alphaPath.push_back(alphaPathPoint);
        robot->betaPath.push_back(betaPathPoint);

        // note make collision segment just two points

        temp[0] = stepNum;
        temp[1] = robot->collisionSegWokXYZ[0][0]; // xAlphaEnd
        robot->roughAlphaX.push_back(temp);
        temp[1] = robot->collisionSegWokXYZ[0][1]; // yAlphaEnd
        robot->roughAlphaY.push_back(temp);
        temp[1] = robot->collisionSegWokXYZ.back()[0]; // xBetaEnd
        robot->roughBetaX.push_back(temp);
        temp[1] = robot->collisionSegWokXYZ.back()[1]; // yBetaEnd
        robot->roughBetaY.push_back(temp);

        return;
    }

    if (atDestination && isEncroaching){
        robot->nudge = true;
    }
    robot->lastStepNum = stepNum;

    // if this robot is not a nudger, and has no other robots nearby
    // try a shortcut move to save computing all move options
    if (!robot->nudge && !isEncroaching){
        // take alpha step toward destination
        if (currAlpha > robot->destinationAlpha){
            nextAlpha = currAlpha - angStep;
            if (nextAlpha < robot->destinationAlpha){
                // don't overshoot
                nextAlpha = robot->destinationAlpha;
            }
        } else if (currAlpha < robot->destinationAlpha){
            nextAlpha = currAlpha + angStep;
            if (nextAlpha > robot->destinationAlpha){
                // don't overshoot
                nextAlpha = robot->destinationAlpha;
            }
        } else {
            // alpha is already at destination, keep it there
            nextAlpha = robot->destinationAlpha;
        }

        // take beta step toward destination
        if (currBeta > robot->destinationBeta){
            nextBeta = currBeta - angStep;
            if (nextBeta < robot->destinationBeta){
                // don't overshoot
                nextBeta = robot->destinationBeta;
            }
        } else if (currBeta < robot->destinationBeta){
            nextBeta = currBeta + angStep;
            if (nextBeta > robot->destinationBeta){
                // don't overshoot
                nextBeta = robot->destinationBeta;
            }
        } else {
            // Beta is already at destination, keep it there
            nextBeta = robot->destinationBeta;
        }

        robot->setAlphaBeta(nextAlpha, nextBeta);

        if (!isCollided(robot->id)){
            // this move worked, keep it and
            // exit early
            // std::cout << "shortcut worked" << std::endl;
            alphaPathPoint[1] = nextAlpha;
            betaPathPoint[1] = nextBeta;
            robot->alphaPath.push_back(alphaPathPoint);
            robot->betaPath.push_back(betaPathPoint);

            temp[0] = stepNum;
            temp[1] = robot->collisionSegWokXYZ[0][0]; // xAlphaEnd
            robot->roughAlphaX.push_back(temp);
            temp[1] = robot->collisionSegWokXYZ[0][1]; // yAlphaEnd
            robot->roughAlphaY.push_back(temp);
            temp[1] = robot->collisionSegWokXYZ.back()[0]; // xBetaEnd
            robot->roughBetaX.push_back(temp);
            temp[1] = robot->collisionSegWokXYZ.back()[1]; // yBetaEnd
            robot->roughBetaY.push_back(temp);
            return;
        }

    }


    // begin looping over all possible moves
    // shuffle move options to ensure they are visited
    // in no particular order
    std::random_shuffle(perturbArray.begin(), perturbArray.end());

    // decide whether we're minimizing phobia
    // or minimizing score

    doPhobia = false;
    greed = 1;

    if (robot->nudge && isEncroaching){
        doPhobia = randomSample() < robot->phobia;
        greed = robot->greed;
        // std::cout << robot->id << " entering avoidance tactics!! " <<  greed << std::endl;
    }


    for (auto dAlphaBeta : perturbArray){
        nextAlpha = currAlpha + dAlphaBeta[0];
        nextBeta = currBeta + dAlphaBeta[1];
        // careful not to overshoot
        if (currAlpha > robot->destinationAlpha and nextAlpha <= robot->destinationAlpha){
            nextAlpha = robot->destinationAlpha;
        }
        if (currAlpha < robot->destinationAlpha and nextAlpha >= robot->destinationAlpha){
            nextAlpha = robot->destinationAlpha;
        }
        if (currBeta > robot->destinationBeta and nextBeta <= robot->destinationBeta){
            nextBeta = robot->destinationBeta;
        }
        if (currBeta < robot->destinationBeta and nextBeta >= robot->destinationBeta){
            nextBeta = robot->destinationBeta;
        }
        // handle limits of travel
        vec2 nextAlphaBeta = handleLimits(currAlpha, currBeta, nextAlpha, nextBeta);
        nextAlpha = nextAlphaBeta[0];
        nextBeta =nextAlphaBeta[1];

        robot->setAlphaBeta(nextAlpha, nextBeta);

        // check for gfa/fiducial collisions

        auto fiducialsColliding = fiducialColliders(robot->id);
        if (fiducialsColliding.size() != 0){
            // std::cout << "fiducial collided" << std::endl;
            break;
        }
        auto gfasColliding = gfaColliders(robot->id);
        if (gfasColliding.size() != 0){
            // std::cout << "gfa collided" << std::endl;
            break;
        }

        // if (isCollided(robot->id)){
        //     // don't consider this a viable option
        //     // move to next option
        //     break;
        // }

        // score is min possible steps till goal
        localEnergy = 0;
        bool robotCollision = false;
        // compute robot's local energy, and check for collision
        for (auto otherRobotID : robot->robotNeighbors){
            auto otherRobot = robotDict[otherRobotID];
            dist2 = dist3D_Segment_to_Segment(
                otherRobot->collisionSegWokXYZ[0], otherRobot->collisionSegWokXYZ[1],
                robot->collisionSegWokXYZ[0], robot->collisionSegWokXYZ[1]
            );

            localEnergy += 1/dist2;

            double collideDist2 = (robot->collisionBuffer + otherRobot->collisionBuffer + maxDisplacement) *
                                  (robot->collisionBuffer + otherRobot->collisionBuffer + maxDisplacement);
            if (dist2 < collideDist2){
                // if collided with another robot stop computing things here
                // and move to the next perturbation
                robotCollision = true;
                break;
            }
        }

        if (robotCollision){
            break;
        }

        if (doPhobia){
            score = localEnergy;
        }
        else {
            score = robot->score();
        }

        if (score < bestScore and greed==1){ //randomSample() < greed){
            bestScore = score;
            bestAlpha = nextAlpha;
            bestBeta = nextBeta;
        }
        else if (score < bestScore and randomSample() < greed){
            bestScore = score;
            bestAlpha = nextAlpha;
            bestBeta = nextBeta;
        }
        // else if (score == bestScore and randomSample() > 0.5){
        //     // if score is same switch to new
        //     // state with 0.5 probability
        //     bestScore = score;
        //     bestAlpha = nextAlpha;
        //     bestBeta = nextBeta;
        // }
    } // end loop over perturbations

    // set alpha beta to best found option
    robot->setAlphaBeta(bestAlpha, bestBeta);
    alphaPathPoint[1] = bestAlpha;
    betaPathPoint[1] = bestBeta;
    robot->alphaPath.push_back(alphaPathPoint);
    robot->betaPath.push_back(betaPathPoint);

    temp[0] = stepNum;
    temp[1] = robot->collisionSegWokXYZ[0][0]; // xAlphaEnd
    robot->roughAlphaX.push_back(temp);
    temp[1] = robot->collisionSegWokXYZ[0][1]; // yAlphaEnd
    robot->roughAlphaY.push_back(temp);
    temp[1] = robot->collisionSegWokXYZ.back()[0]; // xBetaEnd
    robot->roughBetaX.push_back(temp);
    temp[1] = robot->collisionSegWokXYZ.back()[1]; // yBetaEnd
    robot->roughBetaY.push_back(temp);
}





