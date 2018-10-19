// clang++ --std=c++11 -O3 robot.cpp
// vectorizing via = // clang++ --std=c++11 -march=native -O3 robot.cpp
// speeds things up but causes crashes and recommends that you
// include <eigen3/Eigen/StdVector> and add this shit to containers
// that std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>

#include <iostream>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <cmath>
#include <thread>
#include <array>
#include <list>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "utils.h"


// https://stackoverflow.com/questions/28208965/how-to-have-a-class-contain-a-list-of-pointers-to-itself
// https://internal.sdss.org/trac/as4/wiki/FPSLayout
// http://paulbourke.net/geometry/pointlineplane/
// http://geomalgorithms.com/a07-_distance.html#dist3D_Segment_to_Segment()

// constants
const double beta_arm_width = 5.2;
const double buffer_distance = beta_arm_width / 2.0;
const double collide_dist_squared = beta_arm_width * beta_arm_width;
const double collide_dist_squared_shrink = 5 * 5; // collide zone just a bit
const double alpha_arm_len = 7.4;
const double beta_arm_len = 15; // mm to fiber
const double pitch = 22.4; // distance to next nearest neighbor
const double min_reach = beta_arm_len - alpha_arm_len;
const double max_reach = beta_arm_len + alpha_arm_len;

const double ang_step = 0.1; //1.0; // degrees
const int maxPathSteps = (int)(ceil(500.0/ang_step));
// line smoothing factor
const double epsilon =  7 * ang_step;
// from watching things:
// ang_step 0.1 and epsilon = 7 * ang_step seems good
// ang_step 1 and epsilon = 4 * ang_step seems good?

// alpha beta array - the ordered list of moves to try
const double ab_data[] = {
    -ang_step,  ang_step,
            0,  ang_step,
     ang_step,  ang_step,
    -ang_step,         0,
     ang_step,         0,
    -ang_step, -ang_step,
            0, -ang_step,
     ang_step, -ang_step
};

Eigen::Array<double, 8, 2> alphaBetaArr(ab_data);

// define the geometry of the beta arm (a segmented line)
// with origin at beta rotation axis
// x, y, z triplets
// +x is direction along alpha arm
// y is 0 (all points are in plane defined by beta angle dir and +z)
// z is direction along axis of robot rotation (+z is normal to focal plane pointing
// (mostly if we're curved) towards M2)
// goal here is to produce a force field (envelope)

// create a typedef for array of eigen vectors describing
// segments along beta arm. 0,0,0 is point where beta
// axis touches the (bottom of) beta arm

const double b1_data[] = {0, 0, 0};
const double b2_data[] = {0, 0, 7.60};
const double b3_data[] = {6.12, 0, 13.85};
const double b4_data[] = {9.54, 0, 21.90};
const double b5_data[] = {9.54, 0, 30};
// const double b6_data[] = {15.23, 0, 30};
const double b6_data[] = {16.3-buffer_distance, 0, 30};

Eigen::Vector3d b1_v(b1_data); // ignore this guy doesn't contribute +z including in inocous but wastes some computation time
Eigen::Vector3d b2_v(b2_data);
Eigen::Vector3d b3_v(b3_data);
Eigen::Vector3d b4_v(b4_data);
Eigen::Vector3d b5_v(b5_data);
Eigen::Vector3d b6_v(b6_data);

const int betaArmPoints = 5; // should probably make this not a constant?

typedef std::array<Eigen::Vector3d, betaArmPoints> betaGeometry;
betaGeometry betaNeutral = {b2_v, b3_v, b4_v, b5_v, b6_v};

// xyz pos of fiber in beta neutra position
const double fiberNeutral_data[] = {beta_arm_len, 0, 0};
Eigen::Vector3d fiberNeutral(fiberNeutral_data);


// create a vector that translates from beta orgin to alpha origin
// this is just the length of the alpha arm
const double alpha_trans_data[] = {alpha_arm_len, 0, 0};
Eigen::Vector3d alphaTransV(alpha_trans_data);


std::array<double, 2> alphaBetaFromXY(double x, double y){
    // law of cosines at work here...
    double xyMag = hypot(x, y);
    double alphaAngRad = acos(
        (-1*beta_arm_len*beta_arm_len + alpha_arm_len*alpha_arm_len + xyMag*xyMag)/(2*alpha_arm_len*xyMag)
    );
    double gammaAngRad = acos(
        (-1*xyMag*xyMag + alpha_arm_len*alpha_arm_len + beta_arm_len*beta_arm_len)/
        (2*alpha_arm_len*beta_arm_len)
    );
    alphaAngRad = -1*alphaAngRad;
    double betaAngRad = M_PI - gammaAngRad;
    double betaAngDeg = betaAngRad * 180 / M_PI;
    double rotAng = atan2(y, x);
    alphaAngRad = alphaAngRad + rotAng;
    double alphaAngDeg = alphaAngRad * 180 / M_PI;
    while (alphaAngDeg < 0){
        alphaAngDeg += 360;
    }
    std::array<double, 2> outArr;
    outArr[0] = alphaAngDeg;
    outArr[1] = betaAngDeg;
    return outArr;
}


class Robot {
// private:
//     std::array<double, 2> getXYAlongBeta(double);
public:
    int id;
    double xPos, yPos, alpha, beta;
    Eigen::Affine3d betaRot, alphaRot;
    Eigen::Vector3d fiber_XYZ;
    Eigen::Vector3d transXY;
    betaGeometry betaOrientation;
    std::array<double, 2> xyTarget, xyAlphaArm;
    std::vector<Eigen::Vector2d> alphaPath, betaPath;
    std::vector<Eigen::Vector2d> smoothAlphaPath, smoothBetaPath;
    std::vector<Eigen::Vector2d> interpSmoothAlphaPath, interpSmoothBetaPath;
    std::list<Robot *> neighbors;
    Robot (int, double, double);
    void setAlphaBeta (double, double);
    void setAlphaBetaRand();
    void addNeighbor(Robot *);
    void topCollideZone();
    void bottomCollideZone();
    bool isCollided(double collide2 = collide_dist_squared);
    void decollide();
    void setXYUniform();
    void stepTowardFold(int stepNum);
    void pathToFile();
    void smoothPathToFile();
    void ismoothPathToFile();
    void smoothPath();
};

Robot::Robot(int myid, double myxPos, double myyPos) {
    xPos = myxPos;
    yPos = myyPos;
    transXY = Eigen::Vector3d(myxPos, myyPos, 0);
    id = myid;
}

void Robot::pathToFile(){
    FILE * pFile;
    char buffer[50];
    sprintf(buffer, "pathAlpha_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (auto & point : alphaPath){
        fprintf(pFile, "%.2f %.8f\n", point(0), point(1));
    }
    fclose(pFile);

    sprintf(buffer, "pathBeta_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (auto & point : betaPath){
        fprintf(pFile, "%.2f %.8f\n", point(0), point(1));
    }
    fclose(pFile);
}

void Robot::smoothPathToFile(){
    FILE * pFile;
    char buffer[50];
    sprintf(buffer, "smoothpathAlpha_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (auto & point : smoothAlphaPath){
        fprintf(pFile, "%.2f %.8f\n", point(0), point(1));
    }
    fclose(pFile);

    sprintf(buffer, "smoothpathBeta_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (auto & point : smoothBetaPath){
        fprintf(pFile, "%.2f %.8f\n", point(0), point(1));
    }
    fclose(pFile);
}

void Robot::ismoothPathToFile(){
    FILE * pFile;
    char buffer[50];
    sprintf(buffer, "ismoothpathAlpha_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (auto & point : interpSmoothAlphaPath){
        fprintf(pFile, "%.2f %.8f\n", point(0), point(1));
    }
    fclose(pFile);

    sprintf(buffer, "ismoothpathBeta_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (auto & point : interpSmoothBetaPath){
        fprintf(pFile, "%.2f %.8f\n", point(0), point(1));
    }
    fclose(pFile);
}

void Robot::addNeighbor(Robot * rNeighbor){
    neighbors.push_back(rNeighbor);
}


void Robot::setAlphaBeta(double newAlpha, double newBeta){
    alpha = newAlpha;
    beta = newBeta;
    double alphaRad = alpha * M_PI / 180.0;
    double betaRad = beta * M_PI / 180.0;

    // Eigen::Transform<float, 3, Eigen::Affine> t = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    // t.rotate(Eigen::AngleAxisf(betaRad, Eigen::Vector3d::UnitZ()));


    // we begin from beta arm reference frame (thats what betaNeutral is in)
    // and we rotate beta about the origin 1st
    betaRot = Eigen::AngleAxisd(betaRad, Eigen::Vector3d::UnitZ());
    // next create rotation about alpha
    alphaRot = Eigen::AngleAxisd(alphaRad, Eigen::Vector3d::UnitZ());
    // full transform is
    // 1. rotate about beta
    // 2. translate by alpha arm length
    // 3. rotate about alpha
    // 4. translate by xy Pos of positioner
    // result is orientation of positioner in 3d wrt origin of
    // robot grid
    // fullTransform = transXY*alphaRot*alphaTransT*betaRot;
    // fullTransform = (alphaTransT*betaRot).matrix();

    // std::cout << "alpha beta crap" << std::endl;
    for (int ii = 0; ii < betaArmPoints; ii++){
        // first rotate about beta (we start in beta ref frame)
        // next translate in x by the alpha arm length
        // we should be able to figure out how to compute
        // this matrix outside this loop....
        betaOrientation[ii] = transXY + (alphaRot * (alphaTransV + (betaRot * betaNeutral[ii])));

        // std::cout << "test : " << betaOrientation[ii] <<  " : test\n" << std::endl;
    }
    fiber_XYZ = transXY + (alphaRot * (alphaTransV + (betaRot * fiberNeutral)));

}

void Robot::setAlphaBetaRand(){
    double a = randomSample() * 359.99999;
    double b = randomSample() * 180.0;
    setAlphaBeta(a, b);
}

void Robot::setXYUniform(){
    std::array<double, 2> xy = sampleAnnulus(min_reach, max_reach);
    std::array<double, 2> ab = alphaBetaFromXY(xy[0], xy[1]);
    setAlphaBeta(ab[0], ab[1]);
}


bool Robot::isCollided(double collide2){
    bool iAmCollided = false;
    for (Robot * robot : neighbors){
        for (int ii=0; ii<betaArmPoints-1; ii++){
            double dist2 = dist3D_Segment_to_Segment(
                    betaOrientation[ii], betaOrientation[ii+1],
                    robot->betaOrientation[ii], robot->betaOrientation[ii+1]
                );
            if (dist2 < collide2){
                    // std::cout << "dist " << dist2 - collide_dist_squared << std::endl;
                    iAmCollided = true;
                    break;
            }
        }
    }
    return iAmCollided;
}



void Robot::decollide(){
    // randomly replace alpha beta values until collisions vanish
    int ii;
    for (ii=0; ii<300; ii++){
        setXYUniform();
        if (!isCollided()){
            break;
        }
    }
}


void Robot::smoothPath(){
    // smooth a previously generated path
    double interpSmoothAlpha, interpSmoothBeta;
    Eigen::Vector2d atemp, btemp;
    RamerDouglasPeucker(alphaPath, epsilon, smoothAlphaPath);
    RamerDouglasPeucker(betaPath, epsilon, smoothBetaPath);
    // linearly interpolate smooth paths to same step values
    // as computed
    int nDensePoints = alphaPath.size();
    for (int ii=0; ii<nDensePoints; ii++){
        double xVal = alphaPath[ii](0);
        atemp(0) = xVal;
        btemp(0) = xVal;
        interpSmoothAlpha = linearInterpolate(smoothAlphaPath, xVal);
        atemp(1) = interpSmoothAlpha;
        interpSmoothAlphaPath.push_back(atemp);
        interpSmoothBeta = linearInterpolate(smoothBetaPath, xVal);
        btemp(1) = interpSmoothBeta;
        interpSmoothBetaPath.push_back(btemp);
    }

}

void Robot::stepTowardFold(int stepNum){
    double currAlpha = alpha;
    double currBeta = beta;
    Eigen::Vector2d alphaPathPoint;
    Eigen::Vector2d betaPathPoint;
    alphaPathPoint(0) = stepNum;
    betaPathPoint(0) = stepNum;
    if (currBeta==180 and currAlpha==0){
        // done folding don't move
        alphaPathPoint(1) = currAlpha;
        betaPathPoint(1) = currBeta;
        alphaPath.push_back(alphaPathPoint);
        betaPath.push_back(betaPathPoint);
        return;
    }

    // begin trying options pick first that works
    for (int ii=0; ii<alphaBetaArr.rows(); ii++){
        double nextAlpha = currAlpha + alphaBetaArr(ii, 0);
        double nextBeta = currBeta + alphaBetaArr(ii, 1);
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
        setAlphaBeta(nextAlpha, nextBeta);
        if (!isCollided()){
            alphaPathPoint(1) = currAlpha;
            betaPathPoint(1) = currBeta;
            alphaPath.push_back(alphaPathPoint);
            betaPath.push_back(betaPathPoint);
            return;
        }
    }

    // no move options worked,
    // settle for a non-move
    setAlphaBeta(currAlpha, currBeta);
    alphaPathPoint(1) = currAlpha;
    betaPathPoint(1) = currBeta;
    alphaPath.push_back(alphaPathPoint);
    betaPath.push_back(betaPathPoint);
}

bool robotSort(const Robot& robot1, const Robot& robot2){
    if (robot1.beta == 180.0 and robot2.beta < 180.0){
        return false;
    }
    else if (robot1.beta < 180.0 and robot2.beta == 180.0) {
        return true;
    }


    // steps to go
    double r1steps = robot1.alpha + 180 - robot1.beta;
    double r2steps = robot2.alpha + 180 - robot2.beta;
    return (r1steps > r2steps);
}


class RobotGrid {
public:
    int nRobots;
    bool didFail;
    int nSteps;

    double xFocalMax, yFocalMax, xFocalMin, yFocalMin;
    std::list<Robot> allRobots;
    RobotGrid (int);
    void decollide();
    int getNCollisions(double collide2 = collide_dist_squared);
    void toFile(const char*);
    void pathGen();
    void smoothPaths();
    void verifySmoothed();
};

RobotGrid::RobotGrid(int nDia){
    // nDia is number of robots along equator of grid
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
    int printEvery = nSteps / 500;
    int printNum = 0;
    for (int ii = 0; ii < nSteps -1; ii++){
        for (Robot &r : allRobots){
            r.setAlphaBeta(r.interpSmoothAlphaPath[ii](1), r.interpSmoothBetaPath[ii](1));
        }
        nCollisions += getNCollisions(collide_dist_squared_shrink);
        if ((ii % printEvery) == 0){
            sprintf(buffer, "interp_%d.txt", printNum);
            toFile(buffer);
            printNum++;
        }
    }
    std::cout << "interp collisions: " << nCollisions << std::endl;
}



int RobotGrid::getNCollisions(double collide2){
    // return number of collisions found
    int nCollide = 0;
    for (Robot &r : allRobots){
        if (r.isCollided(collide2)) {
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
            r.id, r.xPos, r.yPos, r.alpha, r.beta, r.betaOrientation[0](0), r.betaOrientation[0](1), r.betaOrientation[betaArmPoints-1](0), r.betaOrientation[betaArmPoints-1](1), r.isCollided()
        );
    }
    fclose(pFile);
}

void RobotGrid::pathGen(){
    // first prioritize robots based on their alpha positions
    // robots closest to alpha = 0 are at highest risk with extended
    // betas for getting locked, so try to move those first
    didFail = true;
    int printEvery = maxPathSteps / 500; // want 500 snap shots of path
    int printNum = 0;
    int ii;
    for (ii=0; ii<maxPathSteps; ii++){
        bool allFolded = true;


        // print each step to file
        //----------------------------------------
        // if ((ii % printEvery) == 0){
        //     char buffer[50];
        //     sprintf(buffer, "step_%d.txt", printNum);
        //     toFile(buffer);
        //     printNum++;

        // }

        // allRobots.sort(robotSort);
        for (Robot &r: allRobots){
            // std::cout << "alpha beta " << r.alpha << " " << r.beta << std::endl;
            r.stepTowardFold(ii);
            if (allFolded and r.beta!=180){
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

RobotGrid doOne(){
    RobotGrid rg (25);
    rg.decollide();
    rg.pathGen();
    return rg;
}

void doOneThread(int threadNum){
    int maxIter = 200;
    int seed = threadNum * maxIter;
    char buffer[50];
    for (int ii = 0; ii<maxIter; ii++){
        std::cout << "seed " << seed << std::endl;
        srand(seed);
        RobotGrid rg (25);
        rg.decollide();
        rg.pathGen();
        if (!rg.didFail){
            sprintf(buffer, "success_%04d.txt", seed);
            rg.toFile(buffer);
        }
        seed++;
    }

}

// int main(){
//     int maxIter = 1000;
//     char buffer[50];
//     for (int ii = 0; ii<maxIter; ii++){
//         std::cout << "seed " << ii << std::endl;
//         srand(ii);
//         RobotGrid rg (25);
//         rg.decollide();
//         rg.pathGen();
//         sprintf(buffer, "end_%04d.txt", ii);
//         rg.toFile(buffer);
//     }
// }

// int main()
// {
//     int nThreads = 10;
//     std::thread t[10];
//     clock_t tStart;
//     clock_t tEnd;
//     tStart = clock();
//     for (int i = 0; i<10; ++i){
//         t[i] = std::thread(doOneThread, i);
//     }
//     for (int i=0; i<10; ++i){
//         t[i].join();
//     }
//     tEnd = clock();
//     std::cout << "time took: " << (double)(tEnd - tStart)/CLOCKS_PER_SEC << std::endl;
// }

// int main()
// {
//     std::cout << "max steps " << maxPathSteps << std::endl;
//     // run 500, print out failed grids
//     int nFails = 0;
//     int maxTries = 500;
//     char buffer[50];
//     for (int ii=0; ii<maxTries; ii++){
//         srand(ii);
//         std::cout << "trial " << ii << std::endl;
//         RobotGrid rg = doOne();
//         if(rg.didFail){
//             sprintf(buffer, "fail_%d.txt", ii);
//             rg.toFile(buffer);
//             nFails++;
//         }
//     }
//     std::cout << "nFails " << nFails << std::endl;

// }

int main()
{
    // single run print out robot paths
    srand(0);
    clock_t tStart;
    clock_t tEnd;
    tStart = clock();
    RobotGrid rg = doOne();
    rg.smoothPaths();
    tEnd = clock();
    std::cout << "time took: " << (double)(tEnd - tStart)/CLOCKS_PER_SEC << std::endl;
    for (Robot &robot : rg.allRobots){
        robot.pathToFile();
        robot.smoothPathToFile();
        robot.ismoothPathToFile();
    }
    rg.verifySmoothed();
}


