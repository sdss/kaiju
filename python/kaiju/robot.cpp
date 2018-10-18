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
// #include <eigen3/Eigen/StdVector>


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
// const double top_collide_x1 = 8.187;
// const double beta_head_end = 19.3 - 3; // measured from
// const double top_collide_x2 = 19.3 - 3 - buffer_distance;
const double pitch = 22.4;
const double min_reach = beta_arm_len - alpha_arm_len;
const double max_reach = beta_arm_len + alpha_arm_len;

const double ang_step = 0.1; //1.0; // degrees
const int maxPathSteps = (int)(ceil(500.0/ang_step));
// line smoothing factor
const double epsilon = 10 * ang_step;

// const int maxPathSteps = 20;
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

const double b1_data[] = {0, 0, 0};
const double b2_data[] = {0, 0, 7.60};
const double b3_data[] = {6.12, 0, 13.85};
const double b4_data[] = {9.54, 0, 21.90};
const double b5_data[] = {9.54, 0, 30};
// const double b6_data[] = {15.23, 0, 30};
const double b6_data[] = {16.3-buffer_distance, 0, 30};

Eigen::Vector3d b1_v(b1_data);
Eigen::Vector3d b2_v(b2_data);
Eigen::Vector3d b3_v(b3_data);
Eigen::Vector3d b4_v(b4_data);
Eigen::Vector3d b5_v(b5_data);
Eigen::Vector3d b6_v(b6_data);

const int betaArmPoints = 5;

typedef std::array<Eigen::Vector3d, betaArmPoints> betaGeometry;
betaGeometry betaNeutral = {b2_v, b3_v, b4_v, b5_v, b6_v};

const double fiberNeutral_data[] = {beta_arm_len, 0, 0};
Eigen::Vector3d fiberNeutral(fiberNeutral_data);


// create a vector that translates from beta orgin to alpha origin
// this is just the length of the alpha arm
const double alpha_trans_data[] = {alpha_arm_len, 0, 0};
Eigen::Vector3d alphaTransV(alpha_trans_data);


#define SMALL_NUM   0.00000001 // anything that avoids division overflow
// dot product (3D) which allows vector operations in arguments
#define abs(x)     ((x) >= 0 ? (x) : -(x))   //  absolute value


// create a linear interpolater
double linearInterpolate(std::vector<Eigen::Vector2d> & sparseXYPoints, double xValue){
    Eigen::Vector2d pt1, pt0;
    double yValue;
    int nPoints = sparseXYPoints.size();
    // check that x is in range
    if (xValue < sparseXYPoints[0](0) || xValue > sparseXYPoints[nPoints-1](0)){
        throw std::runtime_error("x outside interpolation range");
    }
    // check if x == last point
    if (xValue == sparseXYPoints[nPoints-1](0)){
        yValue = sparseXYPoints[nPoints-1](1);
    }
    for (int ii = 0; ii < nPoints-1; ii++){
        pt1 = sparseXYPoints[ii+1];
        pt0 = sparseXYPoints[ii];
        if (xValue < pt1(0)){
            yValue = pt0(1) + (pt1(1)-pt0(1)) / (pt1(0) - pt0(0)) * (xValue - pt0(0));
            break;
        }
    }
    return yValue;
}

// dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
//    Input:  two 3D line segments S1 and S2
//    Return: the shortest distance between S1 and S2
double dist3D_Segment_to_Segment(
    Eigen::Vector3d S1_P0, Eigen::Vector3d S1_P1,
    Eigen::Vector3d S2_P0, Eigen::Vector3d S2_P1)
{
    Eigen::Vector3d   u = S1_P1 - S1_P0;
    Eigen::Vector3d   v = S2_P1 - S2_P0;
    Eigen::Vector3d   w = S1_P0 - S2_P0;
    double    a = u.dot(u);         // always >= 0
    double    b = u.dot(v);
    double    c = v.dot(v);         // always >= 0
    double    d = u.dot(w);
    double    e = v.dot(w);
    double    D = a*c - b*b;        // always >= 0
    double    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    double    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

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
    Eigen::Vector3d   dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

    return dP.dot(dP);   // return the closest distance squared
}


// Ramer-Douglas-Peucker for segmentizing paths!
// https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
// https://gist.github.com/TimSC/0813573d77734bcb6f2cd2cf6cc7aa51
// std::vector<Eigen::Vector2d>
double PerpendicularDistance(const Eigen::Vector2d &pt, const Eigen::Vector2d &lineStart, const Eigen::Vector2d &lineEnd)
{
    // copied from dude's github,
    // could be made to use eigen stuff for linalg/norms
    double dx = lineEnd(0) - lineStart(0);
    double dy = lineEnd(1) - lineStart(1);


    //Normalise
    double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
    if(mag > 0.0)
    {
        dx /= mag; dy /= mag;
    }

    double pvx = pt(0) - lineStart(0);
    double pvy = pt(1) - lineStart(1);

    //Get dot product (project pv onto normalized direction)
    double pvdot = dx * pvx + dy * pvy;

    //Scale line direction vector
    double dsx = pvdot * dx;
    double dsy = pvdot * dy;

    //Subtract this from pv
    double ax = pvx - dsx;
    double ay = pvy - dsy;

    return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}

void RamerDouglasPeucker(const std::vector<Eigen::Vector2d> &pointList, double epsilon, std::vector<Eigen::Vector2d> &out)
{
    if(pointList.size()<2)
        throw std::runtime_error("Not enough points to simplify");

    // Find the point with the maximum distance from line between start and end
    double dmax = 0.0;
    size_t index = 0;
    size_t end = pointList.size()-1;
    for(size_t i = 1; i < end; i++)
    {
        double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
        if (d > dmax)
        {
            index = i;
            dmax = d;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if(dmax > epsilon)
    {
        // Recursive call
        std::vector<Eigen::Vector2d> recResults1;
        std::vector<Eigen::Vector2d> recResults2;
        std::vector<Eigen::Vector2d> firstLine(pointList.begin(), pointList.begin()+index+1);
        std::vector<Eigen::Vector2d> lastLine(pointList.begin()+index, pointList.end());
        RamerDouglasPeucker(firstLine, epsilon, recResults1);
        RamerDouglasPeucker(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end()-1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
        if(out.size()<2)
            throw std::runtime_error("Problem assembling output");
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList[0]);
        out.push_back(pointList[end]);
    }
}


double randomSample(){
    // return between 0 and 1
    return static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
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
    bool isCollided();
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
        if (ii==betaArmPoints-1){
        }
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
    std::array<double, 2> xy = sampleAnnulus();
    std::array<double, 2> ab = alphaBetaFromXY(xy[0], xy[1]);
    setAlphaBeta(ab[0], ab[1]);
}


bool Robot::isCollided(){
    bool iAmCollided = false;
    for (Robot * robot : neighbors){
        for (int ii=0; ii<betaArmPoints-1; ii++){
            double dist2 = dist3D_Segment_to_Segment(
                    betaOrientation[ii], betaOrientation[ii+1],
                    robot->betaOrientation[ii], robot->betaOrientation[ii+1]
                );
            if ( dist2 < collide_dist_squared){
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
    int getNCollisions();
    void toFile(const char*);
    void pathGen();
    void smoothPaths();
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
    int ii;
    for (ii=0; ii<maxPathSteps; ii++){
        bool allFolded = true;

        // print each step to file
        //----------------------------------------
        // char buffer[50];
        // sprintf(buffer, "step_%d.txt", ii);
        // toFile(buffer);
        //----------------------------------------

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
}


