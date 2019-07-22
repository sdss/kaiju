# pragma once
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <memory>
#include <vector>
#include <list>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include "betaArm.h"

extern const double alphaLen;
extern const double betaLen;
extern const double maxReach;
extern const double minReach;

class Robot{
public:
    int id;
    int nDecollide = 0;
    int lastStepNum = 0;
    bool targetAssigned = false;
    bool hasApogee;
    double xPos, yPos, alpha, beta;
    double angStep;
    double collisionBuffer = 0;
    Eigen::Array<double, 8, 2> alphaBetaArr;
    Eigen::Affine3d betaRot, alphaRot;
    // Eigen::Vector3d fiber_XYZ;
    Eigen::Vector3d metFiberPos;
    Eigen::Vector3d bossFiberPos;
    Eigen::Vector3d apFiberPos;
    Eigen::Vector3d transXY;
    // betaGeometry betaOrientation, betaModel;
    // std::vector<double> modelRadii;
    // std::array<double, 2> xyTarget, xyAlphaArm;
    std::array<Eigen::Vector3d, 2> betaCollisionSegment;
    std::vector<Eigen::Vector2d> alphaPath, betaPath;
    std::vector<Eigen::Vector2d> smoothAlphaPath, smoothBetaPath; // sparse
    std::vector<Eigen::Vector2d> interpSmoothAlphaPath, interpSmoothBetaPath; // dense
    std::vector<Eigen::Vector2d> interpAlphaX, interpAlphaY, interpBetaX, interpBetaY; // smoothed
    std::vector<Eigen::Vector2d> roughAlphaX, roughAlphaY, roughBetaX, roughBetaY; // jiggly
    std::vector<Eigen::Vector2d> interpCollisions; // boolean points for collided or not
    // std::vector<Robot *> neighbors;
    std::vector<std::shared_ptr<Robot>> neighbors;
    std::vector<Eigen::Vector3d> fiducials;
    Robot (int myid, double myxPos, double myyPos, double myAngStep, bool myHasApogee);
    void setAlphaBeta (double nextAlpha, double nextBeta);
    void setFiberXY (double xFiberGlobal, double yFiberGlobal, int fiberID); // xy in focal plane coord sys
    bool checkFiberXYLocal (double xFiberLocal, double yFiberLocal, int fiberID); // check if robot can reach
    bool checkFiberXYGlobal (double xFiberGlobal, double yFiberGlobal, int fiberID); // check if robot can reach
    void setAlphaBetaRand();
    void addNeighbor(std::shared_ptr<Robot>);
    void addFiducial(std::array<double, 2> fiducial);
    bool isCollided();
    bool isFiducialCollided();
    void decollide();
    void setXYUniform();
    void stepTowardFold(int stepNum);
    void smoothPath(double epsilon);
    void setCollisionBuffer(double newBuffer);
    std::array<double, 2> alphaBetaFromFiberXY(double x, double y, int fiberID);
    // fiberID 0 = metrology
    // fiberID 1 = apogee
    // fiberID 2 = boss
    std::array<double, 2> convFiberXY(double x, double y, int fromFiberID, int toFiberID);
};
