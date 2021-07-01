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

// extern const double alphaLen;
// extern const double betaLen;
// extern const double maxReach;
// extern const double minReach;
// extern const double fiducialBuffer;
// extern const int BOSS_FIBER_ID;
// extern const int AP_FIBER_ID;
// extern const int MET_FIBER_ID;
// extern const double focalZ;

// class Target; // defined elsewhere...

class Robot{
public:
    int id;
    std::string holeID;
    int nDecollide = 0;
    int lastStepNum = 0;
    long assignedTargetID = -1; // -1 indicates no assigned target
    // bool atTarget = false;
    bool hasDestinationAlphaBeta = false;
    bool hasApogee;
    bool hasBoss;
    bool nudge = false;
    double xPos, yPos, alpha, beta, destinationAlpha, destinationBeta; //, targetX, targetY;
    double angStep, minReach, maxReach;
    double collisionBuffer = 0;
    std::vector<double> alphaVel;
    std::vector<double> betaVel;
    std::vector<double> smoothAlphaVel;
    std::vector<double> smoothBetaVel;
    std::vector<double> scoreVec;
    // Eigen::Array<double, 8, 2> alphaBetaArr; // delete this soon
    // Eigen::Affine3d betaRot, alphaRot;

    vec3 basePos;
    vec3 iHat;
    vec3 jHat;
    vec3 kHat;
    vec3 dxyz;
    double alphaLen;
    double alphaOffDeg;
    double betaOffDeg;
    double elementHeight;
    double scaleFac;

    // these don't ever change
    // specified in beta arm coords
    std::array<vec2, 2> collisionSegBetaXY;  // doesn't change
    vec2 metBetaXY;
    vec2 bossBetaXY;
    vec2 apBetaXY;

    // these change with alpha/beta setting
    // specified in wok coords
    std::array<vec3, 2> collisionSegWokXYZ;
    vec3 metWokXYZ;
    vec3 bossWokXYZ;
    vec3 apWokXYZ;

    // Eigen::Vector3d metFiberPos;
    // Eigen::Vector3d targMetFiberPos;
    // Eigen::Vector3d bossFiberPos;
    // Eigen::Vector3d apFiberPos;
    // Eigen::Vector3d transXY;

    // std::array<vec3, 2> betaCollisionSegment;
    std::vector<vec2> alphaPath, betaPath;
    std::vector<vec2> roughAlphaX, roughAlphaY, roughBetaX, roughBetaY; // jiggly
    // std::vector<bool> onTargetVec;
    std::vector<vec2> smoothedAlphaPath, smoothedBetaPath;
    std::vector<vec2> simplifiedAlphaPath, simplifiedBetaPath; // sparse
    std::vector<vec2> interpSimplifiedAlphaPath, interpSimplifiedBetaPath; // dense
    std::vector<vec2> interpAlphaX, interpAlphaY, interpBetaX, interpBetaY; // smoothed
    std::vector<vec2> interpCollisions; // boolean points for collided or not
    std::vector<int> robotNeighbors; // robot IDs in RobotGrid.robotDict may potentially collide
    std::vector<int> fiducialNeighbors; // fiducial IDs in RobotGrid.fiducialDict may potentially collide
    std::vector<long> validTargetIDs; // target IDs in RobotGrid.targetDict that I can reach
    Robot (int id, std::string holeID, vec3 basePos, vec3 iHat, vec3 jHat,
            vec3 kHat, vec3 dxyz, double alphaLen, double alphaOffDeg,
            double betaOffDeg, double elementHeight, double scaleFac, vec2 metBetaXY,
            vec2 bossBetaXY, vec2 apBetaXY,
            std::array<vec2, 2> collisionSegBetaXY, double angStep = 1,
            bool hasApogee = true
    );
    void setAlphaBeta (double alpha, double beta);
    void setDestinationAlphaBeta(double alpha, double beta);
    void setFiberToWokXYZ (vec3 wokXYZ, FiberType fiberType); // xy in focal plane coord sys
    // void setAlphaBetaRand();
    double score(); // metric for how close to target I am
    // double betaWeightedScore(); // metric for how close to target I am
    // double betaScore();
    // double alphaScore();
    void addRobotNeighbor(int robotID);
    void addFiducialNeighbor(int fiducialID);
    // bool isCollided();
    // bool isFiducialCollided();
    // void decollide();
    void setXYUniform();
    vec2 randomXYUniform();
    // void stepTowardFold(int stepNum);
    void simplifyPath(double epsilon);
    void smoothVelocity(int points);
    void setCollisionBuffer(double newBuffer);
    vec2 alphaBetaFromWokXYZ(vec3 wokXYZ, FiberType fiberType);
    vec2 convFiberXY(double x, double y, FiberType fromFiberType, FiberType toFiberType);
    // bool isValidTarget(double x, double y, int fiberID);
    void assignTarget(long targetID);
    void clearAssignment();
    double getMaxReach();
    bool isAssigned();
    // double maxDisplacement();
    // bool canSwapTarget(std::shared_ptr<Robot> robot);
};
