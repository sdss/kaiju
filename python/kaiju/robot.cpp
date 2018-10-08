// clang++ --std=c++11 robot.cpp
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


// https://stackoverflow.com/questions/28208965/how-to-have-a-class-contain-a-list-of-pointers-to-itself
// https://internal.sdss.org/trac/as4/wiki/FPSLayout
// http://paulbourke.net/geometry/pointlineplane/
// http://geomalgorithms.com/a07-_distance.html#dist3D_Segment_to_Segment()

// constants
const double beta_arm_width = 5;
const double buffer_distance = beta_arm_width / 2.0;
const double collide_dist_squared = beta_arm_width * beta_arm_width;
const double alpha_arm_len = 7.4;
const double beta_arm_len = 15; // mm to fiber
const double top_collide_x1 = 8.187;
const double beta_head_end = 19.3 - 3; // measured from
const double top_collide_x2 = 19.3 - 3 - buffer_distance;
const double pitch = 22.4;
const double min_reach = beta_arm_len - alpha_arm_len;
const double max_reach = beta_arm_len + alpha_arm_len;

const double ang_step = 1; //1.0; // degrees
const int maxPathSteps = (int)(ceil(500.0/ang_step));
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
// x, y, z triplets
// x is direction along beta angle
// y is 0 (all points are in plane defined by beta angle dir and +z)
// z is direction along axis of robot (+z is towards M2 when installed at telescope)

// create a typedef for array of eigen vectors describing
// segments along beta arm. 0,0,0 is point where beta
// axis touches the (bottom of) beta arm



const double beta_pt1_data[] = {0, 0, 7.60};
Eigen::Vector3d beta_pt1(beta_pt1_data);
const double beta_pt2_data[] = {6.12, 0, 13.85};
Eigen::Vector3d beta_pt2(beta_pt2_data);
const double beta_pt3_data[] = {9.54, 0, 21.90};
Eigen::Vector3d beta_pt3(beta_pt3_data);
const double beta_pt4_data[] = {9.54, 0, 30};
Eigen::Vector3d beta_pt4(beta_pt4_data);
const double beta_pt5_data[] = {15.23, 0, 30};
Eigen::Vector3d beta_pt5(beta_pt5_data);

// add in point for fiber tips eventually?
typedef std::array<Eigen::Vector3d, 5> betaArmOrientType;

betaArmOrientType betaArmNeutral{ {beta_pt1, beta_pt2, beta_pt3, beta_pt4, beta_pt5} };

#define SMALL_NUM   0.00000001 // anything that avoids division overflow
// dot product (3D) which allows vector operations in arguments
#define abs(x)     ((x) >= 0 ? (x) : -(x))   //  absolute value


// dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
//    Input:  two 3D line segments S1 and S2
//    Return: the shortest distance between S1 and S2
float
dist3D_Segment_to_Segment(Eigen::Vector3d S1_P0, Eigen::Vector3d S1_P1, Eigen::Vector3d S2_P0, Eigen::Vector3d S2_P1)
{
    Vector   u = S1_P1 - S1_P0;
    Vector   v = S2_P1 - S2_P0;
    Vector   w = S1_P0 - S2_P0;
    float    a = u.dot(u);         // always >= 0
    float    b = u.dot(v);
    float    c = v.dot(v);         // always >= 0
    float    d = u.dot(w);
    float    e = v.dot(w);
    float    D = a*c - b*b;        // always >= 0
    float    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    float    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM) { // the lines are almost parallel
        sN = 0.0;         // force using point P0 on segment S1
        sD = 1.0;         // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                 // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d +  b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
    tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

    // get the difference of the two closest points
    Vector   dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

    return dP.dot(dP);   // return the closest distance squared
}

double randomSample(){
    // return between 0 and 1
    return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

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

std::array<double, 2> sampleAnnulus(){
    // random anululs sampling:
    // https://ridlow.wordpress.com/2014/10/22/uniform-random-points-in-disk-annulus-ring-cylinder-and-sphere/
    double rPick = sqrt((max_reach*max_reach - min_reach*min_reach)*randomSample() + min_reach*min_reach);
    double thetaPick = randomSample() * 2 * M_PI;
    std::array<double, 2> outArr;
    outArr[0] = rPick * cos(thetaPick);
    outArr[1] = rPick * sin(thetaPick);
    return outArr;
}

Eigen::MatrixXd getHexPositions(int nDia){
    // returns a 2d array populated with xy positions
    // for a hex packed grid
    // nDia must be odd (not caught)
    int nHex = 0.25*(3*nDia*nDia + 1);
    int nEdge = 0.5*(nDia + 1);
    Eigen::MatrixXd A(nHex, 2);
    double vertShift = sin(60*M_PI/180.0)*pitch;
    double horizShift = cos(60*M_PI/180.0)*pitch;
    int hexInd = 0;
    // start a xStart such that the center of the
    // hex grid is at 0,0
    double xStart = -1*pitch*(nDia - 1.0)/2.0;
    double nextX = xStart;
    double nextY = 0;

    // first fill in equator
    // 0,0 is center
    for (int ii = 0; ii < nDia; ii++){
        A(hexInd,0) = nextX;
        A(hexInd, 1) = nextY;
        nextX += pitch;
        hexInd++;
    }

    // loop over top and bottom rows
    for (int row = 1; row < nEdge; row++){
        nextY = row * vertShift;
        nextX = xStart + row * horizShift;
        for (int jj = 0; jj < nDia - row; jj++){
            A(hexInd, 0) = nextX;
            A(hexInd, 1) = nextY;
            hexInd++;
            A(hexInd, 0) = nextX;
            A(hexInd, 1) = -1*nextY;
            hexInd++;
            nextX += pitch;
        }
    }

    return A;
}

class Robot {
private:
    std::array<double, 2> getXYAlongBeta(double);
public:
    int id;
    double xPos, yPos, alpha, beta;
    betaArmOrientType betaOrientation;
    std::array<double, 2> xyTarget, xyAlphaArm;
    std::vector<double> alphaPath, betaPath;
    std::list<Robot *> neighbors;
    Robot (int, double, double);
    void setAlphaBeta (double, double);
    void setAlphaBetaRand();
    void addNeighbor(Robot *);
    void topCollideZone();
    void bottomCollideZone();
    bool isCollided();
    bool isTopCollided();
    // bool isBottomCollided();
    void decollide();
    void setXYUniform();
    void stepTowardFold(int stepNum);
    void pathToFile();
};

Robot::Robot(int myid, double myxPos, double myyPos) {
    xPos = myxPos;
    yPos = myyPos;
    id = myid;
}

void Robot::pathToFile(){
    FILE * pFile;
    char buffer[50];
    sprintf(buffer, "path_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (int ii=0;ii<alphaPath.size();ii++){
        fprintf(pFile, "%.8f %.8f\n", alphaPath[ii], betaPath[ii]);
    }
    fclose(pFile);
}

void Robot::addNeighbor(Robot * rNeighbor){
    neighbors.push_back(rNeighbor);
}

std::array<double, 2> Robot::getXYAlongBeta(double xBeta){
    // return xy global position from a distance along beta arm
    double y = 0;
    double x = xBeta;

    // first rotate about beta
    double x_b = cosBeta * x - sinBeta * y;
    double y_b = sinBeta * x + cosBeta * y;

    // offset by alpha arm length
    x_b += alpha_arm_len;

    // next rotate about alpha
    double x_a = cosAlpha * x_b - sinAlpha * y_b;
    double y_a = sinAlpha * x_b + cosAlpha * y_b;

    // offset by robot's zero position
    x_a += xPos;
    y_a += yPos;

    std::array<double, 2> outArr;
    outArr[0] = x_a;
    outArr[1] = y_a;
    return outArr;
}

void Robot::setAlphaBeta(double newAlpha, double newBeta){
    alpha = newAlpha;
    beta = newBeta;

    Eigen::AngleAxis<float> rotBeta(betaRad, Eigen::Vector3f::UnitZ());
    Eigen::
    for (int ii=0; ii<betaArmNeutral.size(); ii++){
        betaOrientation[ii] =
    }

    // precompute
    sinAlpha = sin(alphaRad);
    sinBeta = sin(betaRad);
    cosAlpha = cos(alphaRad);
    cosBeta = cos(betaRad);

    // update targetXY pos (end of beta arm)
    xyTarget = getXYAlongBeta(beta_arm_len);
    xyAlphaArm = getXYAlongBeta(0); // could just rotate about alpha...but whatever

    std::array<double, 2> tcpt1 = getXYAlongBeta(top_collide_x1);
    std::array<double, 2> tcpt2 = getXYAlongBeta(top_collide_x2);

    tcCoords[0] = tcpt1[0]; // get rid of tcCoords eventually? they get printed to files
    tcCoords[1] = tcpt1[1];
    tcCoords[2] = tcpt2[0];
    tcCoords[3] = tcpt2[1];
}

void Robot::setAlphaBetaRand(){
    double a = randomSample() * 359.99999;
    double b = randomSample() * 180.0;
    setAlphaBeta(a, b);
}

void Robot::setXYUniform(){
    std::array<double, 2> xy = sampleAnnulus();
    std::array<double, 2> ab = alphaBetaFromXY(xy[0], xy[1]);
    setAlphaBeta(ab[0], ab[1]);
}



bool Robot::isCollided(){
    return isTopCollided();
}

bool Robot::isTopCollided(){
    bool iAmCollided = false;
    for (Robot * robot : neighbors){
        if (dist3D_Segment_to_Segment(tcCoords, robot->tcCoords) < collide_dist_squared){
            iAmCollided = true;
            break;
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

void Robot::stepTowardFold(int stepNum){
    double currAlpha = alpha;
    double currBeta = beta;
    if (currBeta==180 and currAlpha==0){
        // done folding don't move
        alphaPath.push_back(currAlpha);
        betaPath.push_back(currBeta);
        // alphaPath[stepNum] = currAlpha;
        // betaPath[stepNum] = currBeta;
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
            alphaPath.push_back(currAlpha);
            betaPath.push_back(currBeta);
            // alphaPath[stepNum] = nextAlpha;
            // betaPath[stepNum] = nextBeta;
            return;
        }
    }

    // no move options worked,
    // settle for a non-move
    setAlphaBeta(currAlpha, currBeta);
    alphaPath.push_back(currAlpha);
    betaPath.push_back(currBeta);
    // alphaPath[stepNum] = currAlpha;
    // betaPath[stepNum] = currBeta;
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
    int getNCollisions();
    void toFile(const char*);
    void pathGen();
};

RobotGrid::RobotGrid(int nDia){
    // nDia is number of robots along equator of grid
    Eigen::MatrixXd xyHexPos = getHexPositions(nDia);
    nRobots = xyHexPos.rows();

    // determine min/max x/y values in grid

    xFocalMax = xyHexPos.colwise().maxCoeff()(0) + max_reach;
    yFocalMax = xyHexPos.colwise().maxCoeff()(1) + max_reach;
    xFocalMin = xyHexPos.colwise().minCoeff()(0) - min_reach;
    yFocalMin = xyHexPos.colwise().minCoeff()(1) - min_reach;
    // add in robot reach to xyFocalBox
    for (int ii=0; ii<nRobots; ii++){
        Robot robot(ii, xyHexPos(ii, 0), xyHexPos(ii, 1));
        allRobots.push_back(robot);

    }

    // for each robot, give it access to its neighbors
    // and initialze to random alpha betas
    for (Robot &r1 : allRobots){
        r1.setXYUniform();
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
    fprintf(pFile, "# robotID, xPos, yPos, alpha, beta, tcx1, tcy1, tcx2, tcy2, bcx1, bcy1, bcx2, bcy2, isTopCollided, isBottomCollided (step=%d)\n", nSteps);
    for (Robot &r : allRobots){
        fprintf(pFile,
            "%i, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %i, %i\n",
            r.id, r.xPos, r.yPos, r.alpha, r.beta,
            r.tcCoords[0], r.tcCoords[1], r.tcCoords[2], r.tcCoords[3],
            -999.0, -999.0, -999.0, -999.0, // used to be bcCoords
            r.isTopCollided(), 0
        );
    }
    fclose(pFile);
}

void RobotGrid::pathGen(){
    // first prioritize robots based on their alpha positions
    // robots closest to alpha = 0 are at highest risk with extended
    // betas for getting locked, so try to move those first
    didFail = true;
    int ii;
    for (ii=0; ii<maxPathSteps; ii++){
        bool allFolded = true;
        // std::cout << "iter " << ii << std::endl;
        // char buffer[50];
        // sprintf(buffer, "step_%d.txt", ii);
        // toFile(buffer);
        allRobots.sort(robotSort);
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

int main()
{
    int nThreads = 10;
    std::thread t[10];
    clock_t tStart;
    clock_t tEnd;
    tStart = clock();
    for (int i = 0; i<10; ++i){
        t[i] = std::thread(doOneThread, i);
    }
    for (int i=0; i<10; ++i){
        t[i].join();
    }
    tEnd = clock();
    std::cout << "time took: " << (double)(tEnd - tStart)/CLOCKS_PER_SEC << std::endl;
}

// int main()
// {
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
//
// }

// int main()
// {
//     srand(0);
//     clock_t tStart;
//     clock_t tEnd;
//     tStart = clock();
//     RobotGrid rg = doOne();
//     tEnd = clock();
//     std::cout << "time took: " << (double)(tEnd - tStart)/CLOCKS_PER_SEC << std::endl;
//     for (Robot &robot : rg.allRobots){
//         robot.pathToFile();
//     }
// }

