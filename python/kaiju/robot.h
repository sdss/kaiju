# pragma once
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <vector>
#include <list>
#include <array>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "betaArm.h"

// move this somewhere else
const int betaArmPoints = 5; // should probably make this not a constant?

// typedef std::array<Eigen::Vector3d, betaArmPoints> betaGeometry;

class Robot {
public:
    int id;
    double xPos, yPos, alpha, beta;
    Eigen::Affine3d betaRot, alphaRot;
    Eigen::Vector3d fiber_XYZ;
    Eigen::Vector3d transXY;
    betaGeometry betaOrientation, betaModel;
    std::vector<double> modelRadii;
    std::array<double, 2> xyTarget, xyAlphaArm;
    std::vector<Eigen::Vector2d> alphaPath, betaPath;
    std::vector<Eigen::Vector2d> smoothAlphaPath, smoothBetaPath;
    std::vector<Eigen::Vector2d> interpSmoothAlphaPath, interpSmoothBetaPath;
    std::list<Robot *> neighbors;
    Robot (int myid, double myxPos, double myyPos);
    void setAlphaBeta (double nextAlpha, double nextBeta);
    void setAlphaBetaRand();
    void addNeighbor(Robot *);
    bool isCollided(double radiusBuffer=0);
    void decollide();
    void setXYUniform();
    void stepTowardFold(int stepNum);
    void pathToFile();
    void smoothPathToFile();
    void ismoothPathToFile();
    void smoothPath();
};