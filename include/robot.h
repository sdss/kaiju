# pragma once
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <memory>
#include <vector>
#include <list>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "betaArm.h"


class Robot{
public:
    int id;
    int nDecollide = 0;
    int lastStepNum = 0;
    bool targetAssigned = false;
    double xPos, yPos, alpha, beta;
    double ang_step;
    double collisionBuffer = 0;
    Eigen::Array<double, 8, 2> alphaBetaArr;
    Eigen::Affine3d betaRot, alphaRot;
    Eigen::Vector3d fiber_XYZ;
    Eigen::Vector3d transXY;
    betaGeometry betaOrientation, betaModel;
    std::vector<double> modelRadii;
    // std::array<double, 2> xyTarget, xyAlphaArm;
    std::vector<Eigen::Vector2d> alphaPath, betaPath;
    std::vector<Eigen::Vector2d> smoothAlphaPath, smoothBetaPath; // sparse
    std::vector<Eigen::Vector2d> interpSmoothAlphaPath, interpSmoothBetaPath; // dense
    std::vector<Eigen::Vector2d> interpAlphaX, interpAlphaY, interpBetaX, interpBetaY; // smoothed
    std::vector<Eigen::Vector2d> roughAlphaX, roughAlphaY, roughBetaX, roughBetaY; // jiggly
    std::vector<Eigen::Vector2d> interpCollisions; // boolean points for collided or not
    // std::vector<Robot *> neighbors;
    std::vector<std::shared_ptr<Robot>> neighbors;
    Robot (int myid, double myxPos, double myyPos, double myAng_step, betaGeometry myBetaGeom, std::vector<double> myModelRadii);
    void setAlphaBeta (double nextAlpha, double nextBeta);
    void setFiberXY (double xFiberGlobal, double yFiberGlobal); // xy in focal plane coord sys
    bool checkFiberXYLocal (double xFiberLocal, double yFiberLocal); // check if robot can reach
    bool checkFiberXYGlobal (double xFiberGlobal, double yFiberGlobal); // check if robot can reach
    void setAlphaBetaRand();
    void addNeighbor(std::shared_ptr<Robot>);
    bool isCollided();
    void decollide();
    void setXYUniform();
    void stepTowardFold(int stepNum);
    void pathToFile();
    void smoothPathToFile();
    void ismoothPathToFile();
    void smoothPath(double epsilon);
    void setCollisionBuffer(double newBuffer);
};