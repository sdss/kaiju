# pragma once
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <memory>
#include <vector>
#include <list>
#include <array>
#include <map>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "target.h" // has FiberType

extern const double alphaLen;
extern const double betaLen;
extern const double maxReach;
extern const double minReach;
// extern const double fiducialBuffer;
// extern const int BOSS_FIBER_ID;
// extern const int AP_FIBER_ID;
// extern const int MET_FIBER_ID;
extern const double focalZ;

// class Target; // defined elsewhere...

class Robot{
public:
    int id;
    int nDecollide = 0;
    int lastStepNum = 0;
    int assignedTargetID = -1; // -1 indicates no assigned target
    bool hasApogee;
    bool hasBoss;
    double xPos, yPos, alpha, beta;
    double angStep;
    double collisionBuffer = 0;
    std::vector<double> alphaVel;
    std::vector<double> betaVel;
    std::vector<double> smoothAlphaVel;
    std::vector<double> smoothBetaVel;
    Eigen::Array<double, 8, 2> alphaBetaArr;
    Eigen::Affine3d betaRot, alphaRot;
    Eigen::Vector3d metFiberPos;
    Eigen::Vector3d bossFiberPos;
    Eigen::Vector3d apFiberPos;
    Eigen::Vector3d transXY;
    std::array<Eigen::Vector3d, 2> betaCollisionSegment;
    std::vector<Eigen::Vector2d> alphaPath, betaPath;
    std::vector<Eigen::Vector2d> smoothedAlphaPath, smoothedBetaPath;
    std::vector<Eigen::Vector2d> simplifiedAlphaPath, simplifiedBetaPath; // sparse
    std::vector<Eigen::Vector2d> interpSimplifiedAlphaPath, interpSimplifiedBetaPath; // dense
    std::vector<Eigen::Vector2d> interpAlphaX, interpAlphaY, interpBetaX, interpBetaY; // smoothed
    std::vector<Eigen::Vector2d> roughAlphaX, roughAlphaY, roughBetaX, roughBetaY; // jiggly
    std::vector<Eigen::Vector2d> interpCollisions; // boolean points for collided or not
    std::vector<int> robotNeighbors; // robot IDs in RobotGrid.robotDict may potentially collide
    std::vector<int> fiducialNeighbors; // fiducial IDs in RobotGrid.fiducialDict may potentially collide
    std::vector<int> validTargetIDs; // target IDs in RobotGrid.targetDict that I can reach
    Robot (int id, double xPos, double yPos, double angStep = 1, bool hasApogee = true);
    void setAlphaBeta (double nextAlpha, double nextBeta);
    void setFiberXY (double xFiberGlobal, double yFiberGlobal, FiberType fiberType); // xy in focal plane coord sys
    // void setAlphaBetaRand();
    void addRobotNeighbor(int robotID);
    void addFiducialNeighbor(int fiducialID);
    // bool isCollided();
    // bool isFiducialCollided();
    // void decollide();
    void setXYUniform();
    std::array<double, 2> randomXYUniform();
    // void stepTowardFold(int stepNum);
    void simplifyPath(double epsilon);
    void smoothVelocity(int points);
    void setCollisionBuffer(double newBuffer);
    std::array<double, 2> alphaBetaFromFiberXY(double x, double y, FiberType fiberType);
    // fiberID 0 = metrology
    // fiberID 1 = apogee
    // fiberID 2 = boss
    std::array<double, 2> convFiberXY(double x, double y, FiberType fromFiberType, FiberType toFiberType);
    // bool isValidTarget(double x, double y, int fiberID);
    void assignTarget(int targetID);
    void clearAssignment();
    double getMaxReach();
    bool isAssigned();
    // bool canSwapTarget(std::shared_ptr<Robot> robot);
};
