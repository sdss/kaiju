#include <iostream>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <Eigen/Dense>
#include "utils.h"
#include "robotGrid.h"
#include "betaArm.h"

// define constants
const double alpha_arm_len = 7.4;
const double beta_arm_len = 15; // mm to fiber
const double pitch = 22.4; // distance to next nearest neighbor
const double min_reach = beta_arm_len - alpha_arm_len;
const double max_reach = beta_arm_len + alpha_arm_len;

// const double ang_step = 1; // degrees
// const int maxPathStepsGlob = (int)(ceil(700.0/ang_step));
// line smoothing factor
// const double epsilon =  5 * ang_step; // was 7*ang_step for 0.1 step size

// const double min_targ_sep = 8; // mm
const double min_targ_sep = 0; // mm


RobotGrid::RobotGrid(int nDia, double myAng_step, int betaGeomID, int myPrintEvery, double collisionBuffer, double myEpsilon, int seed){
    // nDia is number of robots along equator of grid
    srand(seed);
    smoothCollisions = 0;
    ang_step = myAng_step;
    epsilon = myEpsilon;
    maxPathSteps = (int)(ceil(700.0/ang_step));
    printEvery = myPrintEvery; // default to not printing
    Eigen::MatrixXd xyHexPos = getHexPositions(nDia, pitch);
    nRobots = xyHexPos.rows();

    // get desired betaArm shape
    std::pair<betaGeometry, std::vector<double>> betaPair = getBetaGeom(betaGeomID);

    // determine min/max x/y values in grid

    xFocalMax = xyHexPos.colwise().maxCoeff()(0) + max_reach;
    yFocalMax = xyHexPos.colwise().maxCoeff()(1) + max_reach;
    xFocalMin = xyHexPos.colwise().minCoeff()(0) - min_reach;
    yFocalMin = xyHexPos.colwise().minCoeff()(1) - min_reach;
    // add in robot reach to xyFocalBox
    for (int ii=0; ii<nRobots; ii++){
        Robot robot(ii, xyHexPos(ii, 0), xyHexPos(ii, 1), ang_step, betaPair.first, betaPair.second);
        robot.setCollisionBuffer(collisionBuffer);
        // hack set all alpha betas home
        allRobots.push_back(robot);

    }

    // for each robot, give it access to its neighbors
    // and initialze to random alpha betas
    for (Robot &r1 : allRobots){
        // r1.setAlphaBeta(0,180);
        for (Robot &r2 : allRobots){
            if (r1.id==r2.id){
                continue;
            }
            double dx = r1.xPos - r2.xPos;
            double dy = r1.yPos - r2.yPos;
            double dist = hypot(dx, dy);
            if (dist < (2*pitch+0.1)){ // make this 2*pitch?
                // these robots are neighbors
                r1.addNeighbor(&r2);
            }
        }
    }

    // initialze each robot position
    // ensure minimum target separation
    // assignments keeps track of all previous assignment
    // to check for min target separation
    std::vector<Eigen::Vector2d> assignments;
    for (Robot &r : allRobots){
        while (true) {
            r.setXYUniform();
            Eigen::Vector2d nextAssign;
            nextAssign(0) = r.fiber_XYZ(0);
            nextAssign(1) = r.fiber_XYZ(1);
            bool assignOK = true;
            for (auto &assigned: assignments){
                Eigen::Vector2d diff = nextAssign - assigned;
                if (diff.norm() < min_targ_sep){
                    assignOK = false;
                    // std::cout << "assignment not ok" << std::endl;
                    break; // for loop break
                }
            }
            if (assignOK){
                assignments.push_back(nextAssign);
                break;
            }
        }
    }
}

void RobotGrid::setCollisionBuffer(double newBuffer){
    for (Robot &r : allRobots){
        r.setCollisionBuffer(newBuffer);
    }
}

void RobotGrid::decollide(){
    // iterate over robots and resolve collisions
    while(getNCollisions()){
        for (Robot &r : allRobots){
            if (r.isCollided()){
                r.decollide();
            }
        }
    }

}


void RobotGrid::smoothPaths(){
    for (Robot &r : allRobots){
        r.smoothPath(epsilon);
    }
}

void RobotGrid::verifySmoothed(){
    smoothCollisions = 0;
    char buffer[50];
    int printNum = 0;
    for (int ii = 0; ii < nSteps; ii++){
        for (Robot &r : allRobots){
            r.setAlphaBeta(r.interpSmoothAlphaPath[ii](1), r.interpSmoothBetaPath[ii](1));
            // std::cout << " robot id " << r.id << std::endl;
        }
        smoothCollisions += getNCollisions();
        // std::cout << "n Collisions" << std::endl;
        // std::cout << " ii, printEvery, nsteps alpha path " << ii << " " << printEvery << " " << nSteps << std::endl;

        if (printEvery!=0){
            if ((printEvery==-2 && ii==0) || (printEvery==-1 && ii==nSteps-1) || (ii % printEvery) == 0){
                sprintf(buffer, "interp_%04d.txt", printNum);
                toFile(buffer);
                printNum++;
            }
        }
        // std::cout << "shit balls" << std::endl;
    }
    // std::cout << "interp collisions: " << nCollisions << std::endl;
}



int RobotGrid::getNCollisions(){
    // return number of collisions found
    int nCollide = 0;
    for (Robot &r : allRobots){
        if (r.isCollided()) {
            nCollide++;
        }
    }
    return nCollide;
}

void RobotGrid::toFile(const char* filename){
    FILE * pFile;
    pFile = fopen(filename, "w");
    fprintf(pFile, "# robotID, xPos, yPos, alpha, beta, xAlphaEnd, yAlphaEnd, xBetaEnd, yBetaEnd, isCollided (step=%d)\n", nSteps);
    for (Robot &r : allRobots){
        fprintf(pFile,
            "%i, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %i\n",
            r.id, r.xPos, r.yPos, r.alpha, r.beta, r.betaOrientation[0](0), r.betaOrientation[0](1), r.betaOrientation.back()(0), r.betaOrientation.back()(1), r.isCollided()
        );
    }
    fclose(pFile);
}

void RobotGrid::printStats(const char* filename){
    FILE * pFile;
    pFile = fopen(filename, "w");
    /* print:
    total robots in grid
    success robots in blind target allocation
    number of replacements required to resolve collisions
    number of time steps
    number of robots successfully reaching home
    */
    int totalRobots = 0;
    int allocSuccess = 0;
    int nDecollide = 0;
    int pathSuccess = 0;
    int maxStepsToFullFold = 0;
    for (Robot &r : allRobots){
        totalRobots++;
        if (r.nDecollide == 0){
            allocSuccess++;
        }
        else {
            nDecollide += r.nDecollide;
        }
        if (r.beta==180 and r.alpha==0){
            pathSuccess++;
            if (r.lastStepNum > maxStepsToFullFold){
                maxStepsToFullFold = r.lastStepNum;
            }
        }

    }
    //fprintf(pFile, "#betaGeomID, totalRobots, allocSuccess, nDecollide, pathSuccess, pathSteps\n");
    fprintf(pFile, "%i, %i, %i, %i, %i, %i\n", betaGeomID, totalRobots, allocSuccess, nDecollide, pathSuccess, maxStepsToFullFold);
    fclose(pFile);

}

void RobotGrid::pathGen(){
    // first prioritize robots based on their alpha positions
    // robots closest to alpha = 0 are at highest risk with extended
    // betas for getting locked, so try to move those first
    char buffer[50];
    didFail = true;
    int printNum = 0;
    int ii;
    for (ii=0; ii<maxPathSteps; ii++){
        bool allFolded = true;
        if (printEvery!=0){
            if ((printEvery==-2 && ii==0) || (printEvery==-1 && ii==nSteps-1) || (ii % printEvery) == 0){
                sprintf(buffer, "step_%04d.txt", printNum);
                toFile(buffer);
                printNum++;
            }
        }

        for (Robot &r: allRobots){
            // std::cout << "alpha beta " << r.alpha << " " << r.beta << std::endl;
            r.stepTowardFold(ii);
            if (allFolded and (r.beta!=180 or r.alpha!=0)){
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
    double myX,myY,nX,nY;
    bool swapWorked;
    Robot * rn;
    // int randIndex;
    // int randNeighborIndex;

    // could compile all this stuff into lookup
    // tables but this is working ok

    for (Robot & r : allRobots){
        if (!r.isCollided()){
            continue;  // this robot isn't collided skip it
        }
        swapWorked = false;
        // save current assignment
        myX = r.fiber_XYZ(0);
        myY = r.fiber_XYZ(1);
        std::vector<int> swappableNeighbors;
        for (int ii=0; ii < r.neighbors.size(); ii++){
            rn = r.neighbors[ii];
            nX = rn->fiber_XYZ(0);
            nY = rn->fiber_XYZ(1);
            if (!rn->checkFiberXYGlobal(myX, myY)){
                // neighbor can't reach mine
                // move to next robot
                continue;
            }
            if (!r.checkFiberXYGlobal(nX,nY)){
                // i cant reach neighbor
                // move to next robot
                continue;
            }
            // we can reach eachother, test it
            swappableNeighbors.push_back(ii);
            r.setFiberXY(nX, nY);
            rn->setFiberXY(myX,myY);
            if (!r.isCollided()){
                // no longer collided
                // keep this swap!
                // first one that's good
                swapWorked = true;
                break;
            }
            // swap them back
            r.setFiberXY(myX, myY);
            rn->setFiberXY(nX, nY);
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
