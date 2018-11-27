#include <iostream>
#include <eigen3/Eigen/Dense>
#include "utils.h"
#include "robotGrid.h"
#include "betaArm.h"


RobotGrid::RobotGrid(int nDia, int myMaxPathSteps, int myPrintEvery){
    // nDia is number of robots along equator of grid
    maxPathSteps = myMaxPathSteps;
    printEvery = myPrintEvery; // default to not printing
    Eigen::MatrixXd xyHexPos = getHexPositions(nDia, pitch);
    nRobots = xyHexPos.rows();

    // determine min/max x/y values in grid

    xFocalMax = xyHexPos.colwise().maxCoeff()(0) + max_reach;
    yFocalMax = xyHexPos.colwise().maxCoeff()(1) + max_reach;
    xFocalMin = xyHexPos.colwise().minCoeff()(0) - min_reach;
    yFocalMin = xyHexPos.colwise().minCoeff()(1) - min_reach;
    // add in robot reach to xyFocalBox
    for (int ii=0; ii<nRobots; ii++){
        Robot robot(ii, xyHexPos(ii, 0), xyHexPos(ii, 1));
        // hack set all alpha betas home
        allRobots.push_back(robot);

    }

    // for each robot, give it access to its neighbors
    // and initialze to random alpha betas
    for (Robot &r1 : allRobots){
        r1.setXYUniform();
        // r1.setAlphaBeta(0,180);
        for (Robot &r2 : allRobots){
            if (r1.id==r2.id){
                continue;
            }
            double dx = r1.xPos - r2.xPos;
            double dy = r1.yPos - r2.yPos;
            double dist = hypot(dx, dy);
            if (dist < (pitch+0.1)){
                // these robots are neighbors
                r1.addNeighbor(&r2);
            }
        }
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
        r.smoothPath();
    }
}

void RobotGrid::verifySmoothed(){
    int nCollisions = 0;
    char buffer[50];
    int printNum = 0;
    for (int ii = 0; ii < nSteps; ii++){
        for (Robot &r : allRobots){
            r.setAlphaBeta(r.interpSmoothAlphaPath[ii](1), r.interpSmoothBetaPath[ii](1));
            // std::cout << " robot id " << r.id << std::endl;
        }
        nCollisions += getNCollisions();
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
    std::cout << "interp collisions: " << nCollisions << std::endl;
}



int RobotGrid::getNCollisions(double radiusBuffer){
    // return number of collisions found
    int nCollide = 0;
    for (Robot &r : allRobots){
        if (r.isCollided(radiusBuffer)) {
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

        // allRobots.sort(robotSort);
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


