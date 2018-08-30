// g++ compiler

#include <iostream>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <cmath>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "boost/multi_array.hpp"
#include <boost/range.hpp>

// https://stackoverflow.com/questions/28208965/how-to-have-a-class-contain-a-list-of-pointers-to-itself
// https://internal.sdss.org/trac/as4/wiki/FPSLayout

// constants
const double buffer_distance = 2.5; // robot width is 5
const int points_per_circle = 36*2;
const double alpha_arm_len = 7.4;
const double beta_arm_len = 15; // mm to fiber
const double top_collide_x1 = 8.187;
const double top_collide_x2 = 16.0;
const double bottom_collide_x1 = 0;
const double bottom_collide_x2 = 10.689;
const double pitch = 22.4;
const double min_reach = beta_arm_len - alpha_arm_len;
const double max_reach = beta_arm_len + alpha_arm_len;
const double ang_step = 1; // step 1 degree

typedef double coordinate_type;
typedef boost::geometry::model::d2::point_xy<coordinate_type> point;
typedef boost::geometry::model::polygon<point> polygon;
typedef boost::multi_array<double, 2> nx2Array;
typedef boost::geometry::model::linestring<point> linestring_t;
typedef boost::geometry::model::multi_polygon<polygon> boost_poly;
// typedef nx2Array::index index;

boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(buffer_distance);
boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
boost::geometry::strategy::buffer::side_straight side_strategy;

double randomSample(){
    // return between 0 and 1
    return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

std::array<double, 2> alphaBetaFromXY(double x, double y){
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

nx2Array getHexPositions(int nDia){
    // returns a 2d array populated with xy positions
    // for a hex packed grid
    // nDia must be odd (not caught)
    int nHex = 0.25*(3*nDia*nDia + 1);
    int nEdge = 0.5*(nDia + 1);
    nx2Array A(boost::extents[nHex][2]);
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
        A[hexInd][0] = nextX;
        A[hexInd][1] = nextY;
        nextX += pitch;
        hexInd++;
    }

    // loop over top and bottom rows
    for (int row = 1; row < nEdge; row++){
        nextY = row * vertShift;
        nextX = xStart + row * horizShift;
        for (int jj = 0; jj < nDia - row; jj++){
            A[hexInd][0] = nextX;
            A[hexInd][1] = nextY;
            hexInd++;
            A[hexInd][0] = nextX;
            A[hexInd][1] = -1*nextY;
            hexInd++;
            nextX += pitch;
        }
    }

    return A;
}

class Robot {
private:
    boost_poly getCollideZone(std::array<double, 2>, std::array<double, 2>);
    std::array<double, 2> getXYAlongBeta(double);
public:
    int id;
    double xPos, yPos, alpha, beta;
    double sinAlpha, sinBeta, cosAlpha, cosBeta;
    boost_poly tcz, bcz;
    std::array<double, 4> tcCoords, bcCoords;
    std::array<double, 2> xyTarget, xyAlphaArm;
    std::list<Robot *> neighbors;
    Robot (int, double, double);
    void setAlphaBeta (double, double);
    void setAlphaBetaRand();
    void addNeighbor(Robot *);
    void topCollideZone();
    void bottomCollideZone();
    bool isCollided();
    bool isTopCollided();
    bool isBottomCollided();
    void decollide();
    void setXYUniform();
    void stepTowardFold();
};

Robot::Robot(int myid, double myxPos, double myyPos) {
    xPos = myxPos;
    yPos = myyPos;
    id = myid;
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
    double alphaRad = alpha * M_PI / 180.0;
    double betaRad = beta * M_PI / 180.0;
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
    std::array<double, 2> bcpt2 = getXYAlongBeta(bottom_collide_x2);

    tcCoords[0] = tcpt1[0]; // get rid of tcCoords eventually? they get printed to files
    tcCoords[1] = tcpt1[1];
    tcCoords[2] = tcpt2[0];
    tcCoords[3] = tcpt2[1];

    bcCoords[0] = xyAlphaArm[0];
    bcCoords[1] = xyAlphaArm[1];
    bcCoords[2] = bcpt2[0];
    bcCoords[3] = bcpt2[1];

    tcz = getCollideZone(tcpt1, tcpt2);
    bcz = getCollideZone(xyAlphaArm, bcpt2);
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


boost_poly Robot::getCollideZone(std::array<double, 2> pt1, std::array<double, 2> pt2){
    // pt1 and pt2 describe a line segment
    // create linestring
    linestring_t ls;
    boost::geometry::append(ls, point(pt1[0], pt1[1]));
    boost::geometry::append(ls, point(pt2[0], pt2[1]));

    // Declare output
    boost_poly result;

    // Create the buffer of a linestring
    boost::geometry::buffer(ls, result,
                distance_strategy, side_strategy,
                join_strategy, end_strategy, circle_strategy);
    return result;
}


bool Robot::isCollided(){
    if (isTopCollided()){
        return true;
    }
    else if (isBottomCollided()){
        return true;
    }
    else {
        return false;
    }
}

bool Robot::isTopCollided(){
    bool iAmCollided = false;
    for (Robot * robot : neighbors){
        // do inexpensive check first find midpoint of top
        // collide coord
        double dx = (tcCoords[0] + tcCoords[3])/2.0 - (robot->tcCoords[0] + robot->tcCoords[3])/2.0;
        double dy = (tcCoords[1] + tcCoords[4])/2.0 - (robot->tcCoords[1] + robot->tcCoords[4])/2.0;
        double dist = hypot(dx, dy);
        if (dist > 2*(top_collide_x2 - top_collide_x1 + buffer_distance)){
            continue;
        }
        bool tc = boost::geometry::intersects(tcz, robot->tcz);
        if (tc){
            iAmCollided = true;
            break;
        }
    }
    return iAmCollided;
}

bool Robot::isBottomCollided(){
    bool iAmCollided = false;
    for (Robot * robot : neighbors){
        double dx = (bcCoords[0] + bcCoords[3])/2.0 - (robot->bcCoords[0] + robot->bcCoords[3])/2.0;
        double dy = (bcCoords[1] + bcCoords[4])/2.0 - (robot->bcCoords[1] + robot->bcCoords[4])/2.0;
        double dist = hypot(dx, dy);
        if (dist > 2*(bottom_collide_x2 - bottom_collide_x1 + buffer_distance)){
            continue;
        }
        bool bc = boost::geometry::intersects(bcz, robot->bcz);
        if (bc){
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

void Robot::stepTowardFold(){
    double currAlpha = alpha;
    double currBeta = beta;
    if (currBeta==180 and currAlpha==0){
        // done folding don't move
        return;
    }
    int nMoves = 8; // total number of moves to try
    // build a list of alpha and beta combos to try
    nx2Array alphaBetaArr(boost::extents[nMoves][2]);

    // beta folding
    alphaBetaArr[0][0] = currAlpha - ang_step;
    alphaBetaArr[0][1] = currBeta + ang_step;

    alphaBetaArr[1][0] = currAlpha;
    alphaBetaArr[1][1] = currBeta + ang_step;

    alphaBetaArr[2][0] = currAlpha + ang_step;
    alphaBetaArr[2][1] = currBeta + ang_step;

    // beta static
    alphaBetaArr[3][0] = currAlpha - ang_step;
    alphaBetaArr[3][1] = currBeta;

    alphaBetaArr[4][0] = currAlpha + ang_step;
    alphaBetaArr[4][1] = currBeta;

    // beta unfolding
    alphaBetaArr[5][0] = currAlpha - ang_step;
    alphaBetaArr[5][1] = currBeta - ang_step;

    alphaBetaArr[6][0] = currAlpha;
    alphaBetaArr[6][1] = currBeta - ang_step;

    alphaBetaArr[7][0] = currAlpha + ang_step;
    alphaBetaArr[7][1] = currBeta - ang_step;

    // begin trying options pick first that works
    for (int ii=0; ii<nMoves; ii++){
        double nextAlpha = alphaBetaArr[ii][0];
        double nextBeta = alphaBetaArr[ii][1];
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
            return;
        }
    }

    // no move options worked,
    // settle for a non-move
    setAlphaBeta(currAlpha, currBeta);
}

bool robotSort(const Robot& robot1, const Robot& robot2){
    return (robot1.alpha < robot2.alpha);
}

class RobotGrid {
public:
    int nRobots;
    std::list<Robot> allRobots;
    RobotGrid (int);
    void decollide();
    int getNCollisions();
    void toFile(const char*);
    void pathGen();
};

RobotGrid::RobotGrid(int nDia){
    // nDia is number of robots along equator of grid
    nx2Array xyHexPos = getHexPositions(nDia);
    nRobots = boost::size(xyHexPos);
    // populate list of robots
    for (int ii=0; ii<nRobots; ii++){
        Robot robot(ii, xyHexPos[ii][0], xyHexPos[ii][1]);
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
    fprintf(pFile, "# robotID, xPos, yPos, alpha, beta, tcx1, tcy1, tcx2, tcy2, bcx1, bcy1, bcx2, bcy2, isTopCollided, isBottomCollided\n");
    for (Robot &r : allRobots){
        fprintf(pFile,
            "%i, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %i, %i\n",
            r.id, r.xPos, r.yPos, r.alpha, r.beta,
            r.tcCoords[0], r.tcCoords[1], r.tcCoords[2], r.tcCoords[3],
            r.bcCoords[0], r.bcCoords[1], r.bcCoords[2], r.bcCoords[3],
            r.isTopCollided(), r.isBottomCollided()
        );
    }
    fclose(pFile);
}

void RobotGrid::pathGen(){
    // first prioritize robots based on their alpha positions
    // robots closest to alpha = 0 are at highest risk with extended
    // betas for getting locked, so try to move those first
    int maxIter = 500;
    for (int ii=0; ii<maxIter; ii++){
        std::cout << "iter " << ii << std::endl;
        // char buffer[50];
        // sprintf(buffer, "step_%d.txt", ii);
        // toFile(buffer);
        // allRobots.sort(robotSort);
        for (Robot &r: allRobots){
            r.stepTowardFold();
        }
    }
}

int main()
{
    srand (0);
    RobotGrid rg (25);
    std::cout << "n robots: " << rg.allRobots.size() << std::endl;
    rg.toFile("preCollide.txt");
    std::cout << "nCollisions " << rg.getNCollisions() << std::endl;
    rg.decollide();
    std::cout << "nCollisions " << rg.getNCollisions() << std::endl;
    rg.toFile("postCollide.txt");
    clock_t t;
    t = clock();
    rg.pathGen();
    t = clock() -t;
    std::cout << "took " << float(t)/CLOCKS_PER_SEC << " seconds" << std::endl;
    rg.toFile("pathGen.txt");

}


// example: https://stackoverflow.com/questions/22281962/c11-sorting-list-using-lambda
// #include <iostream>
// #include <list>
// #include <string>

// using namespace std;

// int main()
// {
//    list<pair <string, int>> s = {{"two", 2}, {"one", 1}, {"three", 3}};
//    s.sort( []( const pair<string,int> &a, const pair<string,int> &b ) { return a.second > b.second; } );

// for ( const auto &p : s )
// {
//     cout << p.first << " " << p.second << endl;
// }

// }