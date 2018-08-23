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

typedef double coordinate_type;
typedef boost::geometry::model::d2::point_xy<coordinate_type> point;
typedef boost::geometry::model::polygon<point> polygon;
typedef boost::multi_array<double, 2> hexArray;
typedef boost::geometry::model::linestring<point> linestring_t;
typedef boost::geometry::model::multi_polygon<polygon> boost_poly;
// typedef hexArray::index index;

boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(buffer_distance);
boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
boost::geometry::strategy::buffer::side_straight side_strategy;

hexArray getHexPositions(int nDia){
    // returns a 2d array populated with xy positions
    // for a hex packed grid
    // nDia must be odd (not caught)
    int nHex = 0.25*(3*nDia*nDia + 1);
    int nEdge = 0.5*(nDia + 1);
    hexArray A(boost::extents[nHex][2]);
    double vertShift = sin(60*M_PI/180.0)*pitch;
    double horizShift = cos(60*M_PI/180.0)*pitch;
    int hexInd = 0;
    // start a xStart such that the center of the
    // hex grid is at 0,0
    double xStart = -1*pitch*(nDia - 1.0)/2.0;
    double nextX = xStart;
    double nextY = 0;

    // first fill in equator
    // 0,0 is leftmost
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
    boost_poly collideZone(double, double);
    std::array<double, 4> collideCoords(double, double);
public:
    int id;
    double xPos, yPos, alpha, beta;
    boost_poly tcz, bcz;
    std::array<double, 4> tcCoords, bcCoords;
    std::list<Robot *> neighbors;
    Robot (int, double, double);
    void setAlphaBeta (double, double);
    void setAlphaBetaRand();
    void addNeighbor(Robot *);
    std::array<double, 4> topCollideCoords();
    std::array<double, 4> bottomCollideCoords();
    boost_poly topCollideZone();
    boost_poly bottomCollideZone();
    bool isCollided();
    void decollide();
};

Robot::Robot(int myid, double myxPos, double myyPos) {
    xPos = myxPos;
    yPos = myyPos;
    id = myid;
}

void Robot::addNeighbor(Robot * rNeighbor){
    neighbors.push_back(rNeighbor);
}

void Robot::setAlphaBeta(double newAlpha, double newBeta){
    alpha = newAlpha;
    beta = newBeta;
    tcz = topCollideZone();
    bcz = bottomCollideZone();
}

void Robot::setAlphaBetaRand(){
    double a = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * 359.99999;
    double b = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * 180.0;
    setAlphaBeta(a, b);
}

std::array<double, 4> collideCoords(double betaX1, double betaX2){
    double pt1x, pt1y, pt2x, pt2y, alphaRad, betaRad;
    pt1x = betaX1;
    pt1y = 0;
    pt2x = betaX2;
    pt2y = 0;
    alphaRad = alpha * M_PI / 180.0;
    betaRad = beta * M_PI / 180;

    // first rotate about beta

    double pt1xb = cos(betaRad) * pt1x - sin(betaRad) * pt1y;
    double pt1yb = sin(betaRad) * pt1x + cos(betaRad) * pt1y;
    double pt2xb = cos(betaRad) * pt2x - sin(betaRad) * pt2y;
    double pt2yb = sin(betaRad) * pt2x + cos(betaRad) * pt2y;

    // offset x by alpha arm length
    pt1xb += alpha_arm_len;
    pt2xb += alpha_arm_len;

    // next rotate about alpha
    double pt1xa = cos(alphaRad) * pt1xb - sin(alphaRad) * pt1yb;
    double pt1ya = sin(alphaRad) * pt1xb + cos(alphaRad) * pt1yb;
    double pt2xa = cos(alphaRad) * pt2xb - sin(alphaRad) * pt2yb;
    double pt2ya = sin(alphaRad) * pt2xb + cos(alphaRad) * pt2yb;

    // offset by robot's zero position
    pt1xa += xPos;
    pt2xa += xPos;
    pt1yb += yPos;
    pt2yb += yPos;
}

boost_poly Robot::collideZone(double betaX1, double betaX2){
    double pt1x, pt1y, pt2x, pt2y, alphaRad, betaRad;
    pt1x = betaX1;
    pt1y = 0;
    pt2x = betaX2;
    pt2y = 0;
    alphaRad = alpha * M_PI / 180.0;
    betaRad = beta * M_PI / 180;

    // first rotate about beta

    double pt1xb = cos(betaRad) * pt1x - sin(betaRad) * pt1y;
    double pt1yb = sin(betaRad) * pt1x + cos(betaRad) * pt1y;
    double pt2xb = cos(betaRad) * pt2x - sin(betaRad) * pt2y;
    double pt2yb = sin(betaRad) * pt2x + cos(betaRad) * pt2y;

    // offset x by alpha arm length
    pt1xb += alpha_arm_len;
    pt2xb += alpha_arm_len;

    // next rotate about alpha
    double pt1xa = cos(alphaRad) * pt1xb - sin(alphaRad) * pt1yb;
    double pt1ya = sin(alphaRad) * pt1xb + cos(alphaRad) * pt1yb;
    double pt2xa = cos(alphaRad) * pt2xb - sin(alphaRad) * pt2yb;
    double pt2ya = sin(alphaRad) * pt2xb + cos(alphaRad) * pt2yb;

    // offset by robot's zero position
    pt1xa += xPos;
    pt2xa += xPos;
    pt1yb += yPos;
    pt2yb += yPos;

    // create linestring
    linestring_t ls;
    boost::geometry::append(ls, point(pt1xa, pt1ya));
    boost::geometry::append(ls, point(pt2xa, pt2ya));

    // Declare output
    boost_poly result;

    // Create the buffer of a linestring
    boost::geometry::buffer(ls, result,
                distance_strategy, side_strategy,
                join_strategy, end_strategy, circle_strategy);

    return result;
}

boost_poly Robot::topCollideZone(){
    return collideZone(top_collide_x1, top_collide_x2);
}

boost_poly Robot::bottomCollideZone(){
    return collideZone(bottom_collide_x1, bottom_collide_x2);
}

bool Robot::isCollided(){
    bool iAmCollided = false;
    for (Robot * robot : neighbors){
        bool tc = boost::geometry::intersects(tcz, robot->tcz);
        bool bc = boost::geometry::intersects(bcz, robot->bcz);
        if (tc or bc){
            iAmCollided = true;
            break;
        }
    }
    return iAmCollided;
}

void Robot::decollide(){
    // randomly replace alpha beta values until collisions vanish
    int ii;
    std::cout << "decoliding robot " << id << std::endl;
    for (ii=0; ii<300; ii++){
        setAlphaBetaRand();
        if (isCollided()){
            break;
        }
    }
    std::cout << "decolided robot " << id << " " << ii << " iters" << std::endl;
}

class RobotGrid {
public:
    int nRobots;
    std::list<Robot> allRobots;
    RobotGrid (int);
    void decollide();
    int getNCollisions();
    void toFile();
};

RobotGrid::RobotGrid(int nDia){
    // nDia is number of robots along equator of grid
    hexArray xyHexPos = getHexPositions(25);
    nRobots = boost::size(xyHexPos);
    // populate list of robots
    for (int ii=0; ii<nRobots; ii++){
        Robot robot(ii, xyHexPos[ii][0], xyHexPos[ii][1]);
        allRobots.push_back(robot);
    }
    // for each robot, give it access to its neighbors
    // and initialze to random alpha betas
    for (Robot &r1 : allRobots){
        r1.setAlphaBetaRand();
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
        std::cout << "n collisions " << getNCollisions() << std::endl;
        for (Robot &r : allRobots){
            if (r.isCollided()){
                r.decollide();  // only tries a few times
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

void RobotGrid::toFile(){
    FILE * pFile;
    pFile = fopen("robotGrid.txt", "w");
    fprintf(pFile, "# robotID, xPos, yPos, alpha, beta\n");
    for (Robot &r : allRobots){
        fprintf(pFile, "%i, %.8f, %.8f, %.8f, %.8f\n", r.id, r.xPos, r.yPos, r.alpha, r.beta);
    }
    fclose(pFile);
}

int main()
{
    srand (time(NULL));
    RobotGrid rg (25);
    rg.toFile();
    std::cout << "nCollisions " << rg.getNCollisions() << std::endl;
    // rg.decollide();
    std::cout << "nCollisions " << rg.getNCollisions() << std::endl;

}