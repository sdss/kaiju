#include <iostream>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <Eigen/Dense>
#include "utils.h"
#include "robotGrid.h"
// #include "betaArm.h"

// define constants


const double pitch = 22.4; // distance to next nearest neighbor

// const double ang_step = 1; // degrees
// const int maxPathStepsGlob = (int)(ceil(700.0/ang_step));
// line smoothing factor
// const double epsilon =  5 * ang_step; // was 7*ang_step for 0.1 step size

// const double min_targ_sep = 8; // mm
const double min_targ_sep = 0; // mm


RobotGrid::RobotGrid(int nDia, double myAng_step, double collisionBuffer, double myEpsilon, int seed){
    // nDia is number of robots along equator of grid
    srand(seed);
    smoothCollisions = 0;
    ang_step = myAng_step;
    epsilon = myEpsilon;
    maxPathSteps = (int)(ceil(700.0/ang_step));
    // printEvery = myPrintEvery; // default to not printing
    Eigen::MatrixXd xyHexPos = getHexPositions(nDia, pitch);
    nRobots = xyHexPos.rows();

    // get desired betaArm shape
    // std::pair<betaGeometry, std::vector<double>> betaPair = getBetaGeom(betaGeomID);

    // determine min/max x/y values in grid

    xFocalMax = xyHexPos.colwise().maxCoeff()(0) + maxReach;
    yFocalMax = xyHexPos.colwise().maxCoeff()(1) + maxReach;
    xFocalMin = xyHexPos.colwise().minCoeff()(0) - minReach;
    yFocalMin = xyHexPos.colwise().minCoeff()(1) - minReach;
    // add in robot reach to xyFocalBox
    for (int ii=0; ii<nRobots; ii++){
        auto rptr = std::make_shared<Robot>(ii, xyHexPos(ii, 0), xyHexPos(ii, 1), ang_step);
        rptr->setCollisionBuffer(collisionBuffer);
        // Robot robot(ii, xyHexPos(ii, 0), xyHexPos(ii, 1), ang_step, betaPair.first, betaPair.second);
        // robot.setCollisionBuffer(collisionBuffer);
        // hack set all alpha betas home
        allRobots.push_back(std::move(rptr));

    }

    // for each robot, give it access to its neighbors
    // and initialze to random alpha betas
    for (auto r1 : allRobots){
        // r1.setAlphaBeta(0,180);
        for (auto r2 : allRobots){
            if (r1->id==r2->id){
                continue;
            }
            double dx = r1->xPos - r2->xPos;
            double dy = r1->yPos - r2->yPos;
            double dist = hypot(dx, dy);
            if (dist < (2*pitch+0.1)){ // make this 2*pitch?
                // these robots are neighbors
                r1->addNeighbor(r2);
            }
        }
    }

    // ignoring minimum separation for now
    for (auto r : allRobots){
        r->setXYUniform();
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
    while(getNCollisions()){
        // std::cout << "getting collisions " << std::endl;
        for (auto r : allRobots){
            // std::cout << "decolliding robot " << r.id << std::endl;
            if (r->isCollided()){
                r->decollide();
            }
        }
    }

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


void RobotGrid::optimizeTargets(){
    // warning...only considering boss fibers
    // at the moment
    double myX,myY,nX,nY;
    bool swapWorked;
    // Robot * rn;
    // int randIndex;
    // int randNeighborIndex;

    // could compile all this stuff into lookup
    // tables but this is working ok

    for (auto r : allRobots){
        if (!r->isCollided()){
            continue;  // this robot isn't collided skip it
        }
        swapWorked = false;
        // save current assignment
        myX = r->bossFiberPos(0);
        myY = r->bossFiberPos(1);
        std::vector<int> swappableNeighbors;
        for (int ii=0; ii < r->neighbors.size(); ii++){
            auto rn = r->neighbors[ii];
            nX = rn->bossFiberPos(0);
            nY = rn->bossFiberPos(1);
            if (!rn->checkFiberXYGlobal(myX, myY, 2)){
                // neighbor can't reach mine
                // move to next robot
                continue;
            }
            if (!r->checkFiberXYGlobal(nX, nY, 2)){
                // i cant reach neighbor
                // move to next robot
                continue;
            }
            // we can reach eachother, test it
            swappableNeighbors.push_back(ii);
            r->setFiberXY(nX, nY, 2);
            rn->setFiberXY(myX, myY, 2);
            if (!r->isCollided()){
                // no longer collided
                // keep this swap!
                // first one that's good
                swapWorked = true;
                break;
            }
            // swap them back
            r->setFiberXY(myX, myY, 2);
            rn->setFiberXY(nX, nY, 2);
        }

        // if (!swapWorked and swappableNeighbors.size()>0){
        //     // we didn't find any swap that got rid
        //     // of collision, pick a random valid
        //     // swap for this robot
        //     // set them back,
        //     // swap didn't work
        //     randIndex = rand() % swappableNeighbors.size();
        //     rn = r.neighbors[swappableNeighbors[randIndex]];
        //     nX = rn->fiber_XYZ(0);
        //     nY = rn->fiber_XYZ(1);

        //     r.setFiberXY(nX, nY);
        //     rn->setFiberXY(myX, myY);
        // }


    }


}
