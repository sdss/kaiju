#include <iostream>
#include <stdio.h>      /* printf, scanf, puts, NULL */
// #include <Eigen/Dense>
#include <algorithm>    // std::random_shuffle
#include <chrono>       // std::chrono::system_clock
#include "utils.h"
#include "robotGrid.h"

// define constants
// const double betaCollisionRadius = 1.5; // mm (3mm wide)

// const double pitchRough = 22.4; // distance to next nearest neighbor
const double alphaLenRough = 7.4;
const double betaLenRough = 15;

// const double maxReachCheck2 = 23. * 23.; // maximum reach to check

// const double angStep = 1; // degrees
// const int maxPathStepsGlob = (int)(ceil(700.0/angStep));
// line smoothing factor
// const double epsilon =  5 * angStep; // was 7*angStep for 0.1 step size

// const double min_targ_sep = 8; // mm
// const double min_targ_sep = 0; // mm

// bool sortTargList(std::array<double, 5> targ1, std::array<double, 5> targ2){
//     return targ1[3] > targ2[3];
// }

// bool sortTargList(std::shared_ptr<Target> t1, std::shared_ptr<Target> t2){
//     return t1->priority > t2->priority;
// }

// bool reverseSortTargList(std::shared_ptr<Target> t1, std::shared_ptr<Target> t2){
//     return t1->priority < t2->priority;
// }

// bool sortrobotID(std::shared_ptr<Robot> robot1, std::shared_ptr<Robot> robot2){
//     return robot1->id < robot2->id;
// }

RobotGrid::RobotGrid(double angStep, double collisionBuffer, double epsilon, int seed)
    : angStep(angStep), collisionBuffer(collisionBuffer), epsilon(epsilon), seed(seed)
{
    // nDia is number of robots along equator of grid
    srand(seed);
    // epsilon = myEpsilon;
    // collisionBuffer = myCollisionBuffer;
    // angStep = myAngStep;
    smoothCollisions = 0;
    maxPathSteps = (int)(ceil(1000.0/angStep));
    maxDisplacement = 2*sin(angStep*M_PI/180)*(alphaLenRough+betaLenRough);

    // construct the perturbation list
    for (int ii=-1; ii<2; ii++){
        for (int jj=-1; jj<2; jj++){
            perturbArray.push_back({ii*angStep, jj*angStep});
        }
    }

    // hardcoded and slow, for apo flat wok only!!!
    std::array<vec3, 2> gfa1, gfa2, gfa3, gfa4, gfa5, gfa6;

    gfaCollisionBuffer = 2;

    vec3 gfa1a = {-267.49, 89.15, 143.1};
    vec3 gfa1b = {-210.95, 187.08, 143.1};

    vec3 gfa2a = {-56.54, 276.23, 143.1};
    vec3 gfa2b = {56.54, 276.23, 143.1};

    vec3 gfa3a = {210.95, 187.07, 143.1};
    vec3 gfa3b = {267.49, 89.15, 143.1};

    vec3 gfa4a = {267.49, -89.15, 143.1};
    vec3 gfa4b = {210.95, -187.08, 143.1};

    vec3 gfa5a = {56.54, -276.23, 143.1};
    vec3 gfa5b = {-56.54, -276.23, 143.1};

    vec3 gfa6a = {-210.95, -187.07, 143.1};
    vec3 gfa6b = {-267.49, -89.15, 143.1};

    gfa1[0] = gfa1a;
    gfa1[1] = gfa1b;

    gfa2[0] = gfa2a;
    gfa2[1] = gfa2b;

    gfa3[0] = gfa3a;
    gfa3[1] = gfa3b;

    gfa4[0] = gfa4a;
    gfa4[1] = gfa4b;

    gfa5[0] = gfa5a;
    gfa5[1] = gfa5b;

    gfa6[0] = gfa6a;
    gfa6[1] = gfa6b;

    gfaList[0] = gfa1;
    gfaList[1] = gfa2;
    gfaList[2] = gfa3;
    gfaList[3] = gfa4;
    gfaList[4] = gfa5;
    gfaList[5] = gfa6;

}

void RobotGrid::addRobot(
    int robotID, std::string holeID, vec3 basePos, vec3 iHat, vec3 jHat,
    vec3 kHat, vec3 dxyz, double alphaLen, double alphaOffDeg,
    double betaOffDeg, double elementHeight, double scaleFac, vec2 metBetaXY,
    vec2 bossBetaXY, vec2 apBetaXY,
    std::array<vec2, 2> collisionSegBetaXY,
    bool hasApogee
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
        hasApogee
    );
    robotDict[robotID]->setCollisionBuffer(collisionBuffer);
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
                // std::cout << "add fiducial " << dist << std::endl;
                r1->addFiducialNeighbor(fiducial->id);
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
    collisionBuffer = newBuffer;
    for (auto rPair : robotDict){
        auto r = rPair.second;
        r->setCollisionBuffer(newBuffer);
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
        std::cout << "cannot decolide grid" << std::endl;
        throw std::runtime_error("Unable do decollide robot!!!");
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

void RobotGrid::verifySmoothed(){
    smoothCollisions = 0;
    for (int ii = 0; ii < nSteps; ii++){
        for (auto rPair : robotDict){
            auto r = rPair.second;
            r->setAlphaBeta(r->interpSimplifiedAlphaPath[ii][1], r->interpSimplifiedBetaPath[ii][1]);
            // std::cout << " robot id " << r.id << std::endl;
        }
        smoothCollisions += getNCollisions();
    }
    // std::cout << "interp collisions: " << nCollisions << std::endl;
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
    if (!initialized){
        throw std::runtime_error("Initialize RobotGrid before pathGen");
    }
    for (auto rPair : robotDict){
        auto r = rPair.second;
        // verify that a target alpha beta has been set
        if (!r->hasDestinationAlphaBeta){
            throw std::runtime_error("One or more robots have not received target alpha/beta");
        }
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
        // r->onTargetVec.clear();
    }

}

void RobotGrid::pathGenMDP(double setGreed, double setPhobia){
    // path gen 2 steps towards alpha beta target
    // move greed and phobia to constructor?
    greed = setGreed;
    phobia = setPhobia;
    algType = MDP;
    clearPaths();
    didFail = true;
    int ii;
    std::vector<int> robotIDs;
    for (auto rPair : robotDict){
        robotIDs.push_back(rPair.first);
    }

    for (ii=0; ii<maxPathSteps; ii++){
        // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::random_shuffle(robotIDs.begin(), robotIDs.end());
        bool allAtTarget = true;
        for (auto robotID : robotIDs){
            auto r = robotDict[robotID];
            // std::cout << "path gen " << r.betaOrientation.size() << " " << r.betaModel.size() << std::endl;
            // std::cout << "alpha beta " << r.alpha << " " << r.beta << std::endl;
            stepMDP(r, ii);
            r->scoreVec.push_back(r->score());
            if (r->score()!=0) {
                // could just check the last elemet in onTargetVec? same thing.
                // or use robot->score
                allAtTarget = false;
            }
        }

        if (allAtTarget){
            // std::cout << "all at target" << std::endl;
            didFail = false;
            break;
        }
    }

    nSteps = ii+1;
}

void RobotGrid::pathGenGreedy(){
    // path gen 2 steps towards alpha beta target
    clearPaths();
    didFail = true;
    greed = 1;
    phobia = 0;
    algType = Greedy;
    int ii;
    for (ii=0; ii<maxPathSteps; ii++){

        bool allAtTarget = true;

        for (auto rPair : robotDict){
            auto r = rPair.second;
            // std::cout << "path gen " << r.betaOrientation.size() << " " << r.betaModel.size() << std::endl;
            // std::cout << "alpha beta " << r.alpha << " " << r.beta << std::endl;
            stepGreedy(r, ii);
            r->scoreVec.push_back(r->score());
            if (r->score()!=0) {
                // could just check the last elemet in onTargetVec? same thing.
                // or use robot->score
                allAtTarget = false;
            }
        }

        if (allAtTarget){
            // std::cout << "all at target" << std::endl;
            didFail = false;
            break;
        }
    }

    nSteps = ii+1;
}

void RobotGrid::pathGenEscape(double deg2move){
    // first prioritize robots based on their alpha positions
    // robots closest to alpha = 0 are at highest risk with extended
    // betas for getting locked, so try to move those first
    // int pathPad = 20 / (float)angStep;
    clearPaths();
    int ii;
    double steps2move = deg2move / angStep;
    for (ii=0; ii<steps2move; ii++){

        for (auto rPair : robotDict){
            auto r = rPair.second;
            // std::cout << "path gen " << r.betaOrientation.size() << " " << r.betaModel.size() << std::endl;
            // std::cout << "alpha beta " << r.alpha << " " << r.beta << std::endl;
            stepDecollide(r, ii);
        }

        // if (getNCollisions()==0){
        //     break;
        // }
    }

    nSteps = ii;
}



// void RobotGrid::pathGen(){
//     // first prioritize robots based on their alpha positions
//     // robots closest to alpha = 0 are at highest risk with extended
//     // betas for getting locked, so try to move those first
//     // int pathPad = 20 / (float)angStep;
//     clearPaths();
//     algType = Fold;
//     didFail = true;
//     greed = -1;
//     phobia = -1;
//     int ii;
//     for (ii=0; ii<maxPathSteps; ii++){
//         bool allFolded = true;

//         for (auto rPair : robotDict){
//             auto r = rPair.second;
//             // std::cout << "path gen " << r.betaOrientation.size() << " " << r.betaModel.size() << std::endl;
//             // std::cout << "alpha beta " << r.alpha << " " << r.beta << std::endl;
//             stepTowardFold(r, ii);
//             if (r->beta!=180 or r->alpha!=0) { // stop when beta = 180} or r.alpha!=0)){
//                 allFolded = false;
//             }
//         }

//         if (allFolded){
//             didFail = false;
//             break;
//         }
//     }

//     nSteps = ii+1;
// }

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

    // first a quick position cut
    // quick position cut fails near edges, get rid of it
    // double targDist2 = (target->x - robot->xPos) * (target->x - robot->xPos) +
    //   (target->y - robot->yPos) * (target->y - robot->yPos) ;
    // if (targDist2 > maxReachCheck2) {
    //   return false;
    // }

    auto ab = robot->alphaBetaFromWokXYZ(target->xyzWok, target->fiberType);
    // check alpha beta valid
    if (std::isnan(ab[0]) or std::isnan(ab[1])){
        return false;
    }
    // check alpha beta in range
    // if (ab[0]<0 or ab[0]>=360){
    //     return false;
    // }
    // if (ab[1]<0 or ab[1]>180){
    //     return false;
    // }

    // save current alpha beta
    double savedAlpha = robot->alpha;
    double savedBeta = robot->beta;
    robot->setAlphaBeta(ab[0], ab[1]);
    auto collidedFiducials = fiducialColliders(robotID);
    if (collidedFiducials.size() != 0){
        // this target interferes with a fiducial which is immobile
        robot->setAlphaBeta(savedAlpha, savedBeta);
        return false;
    }
    // reset alpha beta
    robot->setAlphaBeta(savedAlpha, savedBeta);
    return true;
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
    return gfaColliders(robotID);
}

std::tuple<bool, bool, std::vector<int>> RobotGrid::isCollidedWithAssigned(int robotID){
  bool collided, fiducial_collided;
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
  collided = (fiducialsColliding.size() != 0) || (assignedRobotsColliding.size() != 0);
  fiducial_collided = (fiducialsColliding.size() != 0);
  return std::make_tuple(collided, fiducial_collided, assignedRobotsColliding);
}

void RobotGrid::homeRobot(int robotID){
    unassignRobot(robotID);
    auto robot = robotDict[robotID];
		robot->setAlphaBeta(0., 180.);
}

std::tuple<bool, bool, std::vector<int>> RobotGrid::wouldCollideWithAssigned(int robotID, long targID){
  long currentTargetID;
  int currentRobotID;
  std::tuple<bool, bool, std::vector<int>> result;
  
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


bool RobotGrid::neighborEncroachment(std::shared_ptr<Robot> robot1){
    // score, separation2
    // look ahead and see robots getting close
    double dist2;
    double minDist = 2*collisionBuffer + 3*maxDisplacement;

    // std::cout << "check ne! " << std::endl;
    // turn off neighbor encroachment if robot is disables
    if (robot1->isOffline){
        return false;
    }

    // check collisions with neighboring robots
    for (auto otherRobotID : robot1->robotNeighbors){
        auto robot2 = robotDict[otherRobotID];
        // squared distance returned
        dist2 = dist3D_Segment_to_Segment(
                robot2->collisionSegWokXYZ[0], robot2->collisionSegWokXYZ[1],
                robot1->collisionSegWokXYZ[0], robot1->collisionSegWokXYZ[1]
            );
        if (dist2 < (minDist*minDist)){

            // std::cout << "neighbor encroachment! " << std::endl;

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
    // std::cout << "robot max displace " << maxDisplacement << std::endl;
    for (auto otherRobotID : robot1->robotNeighbors){
        auto robot2 = robotDict[otherRobotID];
        // squared distance returned
        dist2 = dist3D_Segment_to_Segment(
                robot2->collisionSegWokXYZ[0], robot2->collisionSegWokXYZ[1],
                robot1->collisionSegWokXYZ[0], robot1->collisionSegWokXYZ[1]
            );
        dist = sqrt(dist2);
        // collideDist2 = (2*collisionBuffer)*(2*collisionBuffer);
        if (dist < (2*collisionBuffer + maxDisplacement)){
            // std::cout << "dist " << dist2 - collide_dist_squared << std::endl;
            collidingNeighbors.push_back(robot2->id);
        }

    }
    return collidingNeighbors;
}

std::vector<int> RobotGrid::fiducialColliders(int robotID){

    std::vector<int> collidingNeighbors;
    auto robot = robotDict[robotID];
    // std::cout << "isFiducialCollided" << std::endl;
    double dist2, collideDist2;
    // std::cout << "n fiducials " << fiducials.size() << std::endl;
    for (auto fiducialID : robot->fiducialNeighbors){
        auto fiducial = fiducialDict[fiducialID];
        // squared distance
        // std::array<double, 2> xyCoord = {fiducial->x, fiducial->y};
        // vec3 xyzCoord = {fiducial->x, fiducial->y, focalZ};
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
    return collidingNeighbors;

}

bool RobotGrid::gfaColliders(int robotID){

    auto robot = robotDict[robotID];
    // std::cout << "isFiducialCollided" << std::endl;
    double dist2, collideDist2;
    // std::cout << "n fiducials " << fiducials.size() << std::endl;
    for (auto gfaSeg : gfaList){
        dist2 = dist3D_Segment_to_Segment(
                gfaSeg[0], gfaSeg[1],
                robot->collisionSegWokXYZ[0], robot->collisionSegWokXYZ[1]
            );

        collideDist2 =  (robot->collisionBuffer+gfaCollisionBuffer) *
                        (robot->collisionBuffer+gfaCollisionBuffer);

        if (dist2 < collideDist2){
            std::cout << "GFA Collision!" << std::endl;
            return true;
        }
    }
    return false;

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

// bool RobotGrid::replaceNearFold(int robotID){
//     // return true if worked
//     // robot gets new alpha beta position
//     // if return is false robot is unchanged
//     auto robot = robotDict[robotID];
//     auto currAlpha = robot->alpha;
//     auto currBeta = robot->beta;
//     // attempt to keep beta as small as possible
//     // loop over it first
//     double betaProbe = 180;
//     double stepSize = 0.05;
//     while (betaProbe > 0){
//         double alphaProbe = 0;
//         while (alphaProbe < 360){
//             robot->setAlphaBeta(alphaProbe, betaProbe);
//             if (!isCollided(robotID)){
//                 return true;
//             }
//             alphaProbe = alphaProbe + stepSize;
//         }
//         betaProbe = betaProbe - stepSize;
//     }
//     // couldn't find shit!
//     robot->setAlphaBeta(currAlpha, currBeta);
//     return false;
// }


void RobotGrid::decollideRobot(int robotID){
    // remove assigned target if present
    // std::cout << "decolliding robot " << robot->id << std::endl;
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
    // are we still collided?
    // if (isCollided(robot)){
    //     throw std::runtime_error("Unable do decollide robot!!!");
    // }
}

std::vector<int> RobotGrid::deadlockedRobots(){
    std::vector<int> deadlockedRobotIDs;


    // auto r362 = robotDict[362];
    // double minDist = 4*collisionBuffer*4*maxDisplacement;
    // std::cout << "minDist " << minDist << std::endl;
    // for (auto otherRobotID : r362->robotNeighbors){
    //     auto robot2 = robotDict[otherRobotID];
    //     // squared distance returned
    //     double dist2 = dist3D_Segment_to_Segment(
    //             robot2->collisionSegWokXYZ[0], robot2->collisionSegWokXYZ[1],
    //             r362->collisionSegWokXYZ[0], r362->collisionSegWokXYZ[1]
    //         );
    //     std::cout << otherRobotID <<": " << dist2 << std::endl;
    // }

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
    // std::cout << "beta orientation size: " << betaOrientation.size() << std::endl;
    // std::cout << "beta model size: " << betaModel.size() << std::endl;
    temp[1] = robot->collisionSegWokXYZ[0][0]; // xAlphaEnd
    // std::cout << "step toward fold " << ii << std::endl;

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

    // so scope really fucked me on this one?
    // lots of returns fixed it.
    double dist2, collideDist2;
    // check collisions with neighboring robots
    double minDist = 1e9;  // to be minimized
    auto robot1 = robotDict[robotID];


    // search through robot neighbors for min dist
    // std::cout << "robot max displace " << maxDisplacement << std::endl;
    for (auto otherRobotID : robot1->robotNeighbors){
        auto robot2 = robotDict[otherRobotID];
        // squared distance returned
        dist2 = dist3D_Segment_to_Segment(
                robot2->collisionSegWokXYZ[0], robot2->collisionSegWokXYZ[1],
                robot1->collisionSegWokXYZ[0], robot1->collisionSegWokXYZ[1]
            );

        if (dist2 < minDist){
            minDist = dist2;
        }
    }

    for (auto fiducialID : robot1->fiducialNeighbors){
        auto fiducial = fiducialDict[fiducialID];
        // squared distance
        // std::array<double, 2> xyCoord = {fiducial->x, fiducial->y};
        // vec3 xyzCoord = {fiducial->x, fiducial->y, focalZ};
        dist2 = dist3D_Point_to_Segment(
                fiducial->xyzWok, robot1->collisionSegWokXYZ[0],
                robot1->collisionSegWokXYZ[1]
                );
        if (dist2 < minDist){
            minDist = dist2;
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
    // bestScore = robot->score() + 1/closestApproach2(robot->id);
    // bestScore = 1e16;

    vec2 alphaPathPoint;
    vec2 betaPathPoint;
    vec2 temp;

    alphaPathPoint[0] = stepNum;
    betaPathPoint[0] = stepNum;


    if (robot->score()==0){
        // at target don't move
        alphaPathPoint[1] = currAlpha;
        betaPathPoint[1] = currBeta;
        robot->alphaPath.push_back(alphaPathPoint);
        robot->betaPath.push_back(betaPathPoint);
        // robot->onTargetVec.push_back(true);

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
        // double encroachment = 0;

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
    // bestEncroachment = 1e16; // to be minimized
    // double bestCost = 1e16; // to be minimized
    // bestScore = robot->score() + 1/closestApproach2(robot->id);
    // bestScore = 1e16;

    vec2 alphaPathPoint;
    vec2 betaPathPoint;
    vec2 temp;

    alphaPathPoint[0] = stepNum;
    betaPathPoint[0] = stepNum;

    // nextAlpha, nextBeta, local energy, score
    // std::vector<std::array<double, 4>> stateOptions;

    if (robot->score()==0 and !neighborEncroachment(robot)){
        // done folding no one knocking don't move
        alphaPathPoint[1] = currAlpha;
        betaPathPoint[1] = currBeta;
        robot->alphaPath.push_back(alphaPathPoint);
        robot->betaPath.push_back(betaPathPoint);
        // robot->onTargetVec.push_back(true);

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

    doPhobia = randomSample() < phobia;


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
        // score is min possible steps till goal
        localEnergy = 0;
        bool isCollided = false;

        // compute robot's local energy, and check for collision
        for (auto otherRobotID : robot->robotNeighbors){
            auto otherRobot = robotDict[otherRobotID];
            dist2 = dist3D_Segment_to_Segment(
                otherRobot->collisionSegWokXYZ[0], otherRobot->collisionSegWokXYZ[1],
                robot->collisionSegWokXYZ[0], robot->collisionSegWokXYZ[1]
            );

            localEnergy += 1/dist2;

            if (dist2 < (2*collisionBuffer+maxDisplacement)*(2*collisionBuffer+maxDisplacement)){
                // this is not a viable move option
                // go on to next try
                isCollided = true;
                if (otherRobot->score() < robot->score()){
                    otherRobot->nudge = true;
                }
            }
        }

        if (isCollided){
            // don't consider this a viable option
            // move to next option
            break;
        }

        if (doPhobia){
            score = localEnergy;
        }
        else {
            score = robot->score();
        }

        if (score < bestScore and randomSample() < greed){
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
    robot->nudge = false;
}


// void RobotGrid::pathGen(){
//     // first prioritize robots based on their alpha positions
//     // robots closest to alpha = 0 are at highest risk with extended
//     // betas for getting locked, so try to move those first
//     // int pathPad = 20 / (float)angStep;
//     if (!initialized){
//         throw std::runtime_error("Initialize RobotGrid before pathGen");
//     }
//     didFail = true;
//     for (auto rPair : robotDict){
//         auto r = rPair.second;
//         // clear any existing path
//         r->alphaPath.clear();
//         r->betaPath.clear();
//         r->simplifiedAlphaPath.clear();
//         r->simplifiedBetaPath.clear(); // sparse
//         r->interpSimplifiedAlphaPath.clear();
//         r->interpSimplifiedBetaPath.clear(); // dense
//         r->smoothedAlphaPath.clear();
//         r->smoothedBetaPath.clear();
//         r->smoothAlphaVel.clear();
//         r->smoothBetaVel.clear();
//         r->interpAlphaX.clear();
//         r->interpAlphaY.clear();
//         r->interpBetaX.clear();
//         r->interpBetaY.clear(); // smoothed
//         r->roughAlphaX.clear();
//         r->roughAlphaY.clear();
//         r->roughBetaX.clear();
//         r->roughBetaY.clear();
//     }
//     int ii;
//     for (ii=0; ii<maxPathSteps; ii++){
//         bool allFolded = true;

//         for (auto rPair : robotDict){
//             auto r = rPair.second;
//             // std::cout << "path gen " << r.betaOrientation.size() << " " << r.betaModel.size() << std::endl;
//             // std::cout << "alpha beta " << r.alpha << " " << r.beta << std::endl;
//             stepTowardFold(r, ii);
//             if (r->beta!=180 or r->alpha!=0) { // stop when beta = 180} or r.alpha!=0)){
//                 allFolded = false;
//             }
//         }

//         if (allFolded){
//             didFail = false;
//             break;
//         }
//     }

//     nSteps = ii;
// }


// void RobotGrid::stepTowardFold(std::shared_ptr<Robot> robot, int stepNum){
//     double currAlpha = robot->alpha;
//     double currBeta = robot->beta;
//     vec2 alphaPathPoint;
//     vec2 betaPathPoint;
//     vec2 temp;
//     alphaPathPoint[0] = stepNum;
//     betaPathPoint[0] = stepNum;

//     if (currBeta==180 and currAlpha==0){
//         // done folding don't move
//         alphaPathPoint[1] = currAlpha;
//         betaPathPoint[1] = currBeta;
//         robot->alphaPath.push_back(alphaPathPoint);
//         robot->betaPath.push_back(betaPathPoint);

//         temp[0] = stepNum;
//         temp[1] = robot->collisionSegWokXYZ[0][0]; // xAlphaEnd
//         robot->roughAlphaX.push_back(temp);
//         temp[1] = robot->collisionSegWokXYZ[0][1]; // yAlphaEnd
//         robot->roughAlphaY.push_back(temp);
//         temp[1] = robot->collisionSegWokXYZ.back()[0]; // xBetaEnd
//         robot->roughBetaX.push_back(temp);
//         temp[1] = robot->collisionSegWokXYZ.back()[1]; // yBetaEnd
//         robot->roughBetaY.push_back(temp);
//         // robot->onTargetVec.push_back(true);

//         return;
//     }
//     // this is for keeping track of last step
//     // only updates if robot hasn't reached fold
//     robot->lastStepNum = stepNum;
//     // begin trying options pick first that works
//     for (int ii=0; ii<robot->alphaBetaArr.rows(); ii++){
//         double nextAlpha = currAlpha + robot->alphaBetaArr(ii, 0);
//         double nextBeta = currBeta + robot->alphaBetaArr(ii, 1);
//         if (nextAlpha > 360){
//             nextAlpha = 360;
//         }
//         if (nextAlpha < 0){
//             nextAlpha = 0;
//         }
//         if (nextBeta > 180){
//             nextBeta = 180;
//         }
//         if (nextBeta < 0){
//             nextBeta = 0;
//         }
//         // if next choice results in no move skip it
//         // always favor a move
//         if (nextBeta==currBeta and nextAlpha==currAlpha){
//             continue;
//         }
//         robot->setAlphaBeta(nextAlpha, nextBeta);
//         if (!isCollided(robot->id)){
//             alphaPathPoint[1] = nextAlpha;
//             betaPathPoint[1] = nextBeta;
//             robot->alphaPath.push_back(alphaPathPoint);
//             robot->betaPath.push_back(betaPathPoint);

//             // add alpha/beta xy points

//             temp[0] = stepNum;
//             // std::cout << "beta orientation size: " << betaOrientation.size() << std::endl;
//             // std::cout << "beta model size: " << betaModel.size() << std::endl;
//             temp[1] = robot->collisionSegWokXYZ[0][0]; // xAlphaEnd
//             // std::cout << "step toward fold " << ii << std::endl;

//             robot->roughAlphaX.push_back(temp);

//             temp[1] = robot->collisionSegWokXYZ[0][1]; // yAlphaEnd
//             robot->roughAlphaY.push_back(temp);
//             temp[1] = robot->collisionSegWokXYZ.back()[0]; // xBetaEnd
//             robot->roughBetaX.push_back(temp);
//             temp[1] = robot->collisionSegWokXYZ.back()[1]; // yBetaEnd
//             robot->roughBetaY.push_back(temp);

//             return;
//         }
//     }

//     // no move options worked,
//     // settle for a non-move
//     robot->setAlphaBeta(currAlpha, currBeta);
//     alphaPathPoint[1] = currAlpha;
//     betaPathPoint[1] = currBeta;
//     robot->alphaPath.push_back(alphaPathPoint);
//     robot->betaPath.push_back(betaPathPoint);

//     // add alpha/beta xy points
//     // vec2 temp;
//     temp[0] = stepNum;
//     temp[1] = robot->collisionSegWokXYZ[0][0]; // xAlphaEnd
//     robot->roughAlphaX.push_back(temp);
//     temp[1] = robot->collisionSegWokXYZ[0][1]; // yAlphaEnd
//     robot->roughAlphaY.push_back(temp);
//     temp[1] = robot->collisionSegWokXYZ.back()[0]; // xBetaEnd
//     robot->roughBetaX.push_back(temp);
//     temp[1] = robot->collisionSegWokXYZ.back()[1]; // yBetaEnd
//     robot->roughBetaY.push_back(temp);

// }





