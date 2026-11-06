# pragma once
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <memory>
#include <vector>
#include <list>
#include <array>
#include <map>
#include <algorithm>
#include "target.h" // has FiberType


class Robot{
public:
    int id;
    std::string holeID;
    int nDecollide = 0;
    int lastStepNum = 0;
    long assignedTargetID = -1; // -1 indicates no assigned target
    bool hasDestinationAlphaBeta = false;
    bool hasApogee;
    bool hasBoss;
    bool lefthanded;
    bool nudge = false;
    bool isOffline = false;
    bool fiducialWatch = false;
    double xPos, yPos, alpha, beta, alphaInit, betaInit, destinationAlpha, destinationBeta; //, targetX, targetY;
    double angStep, minReach, maxReach;
    double collisionBuffer = 0;
    double greed = 1;
    double phobia = 0;
    std::vector<double> alphaVel;
    std::vector<double> betaVel;
    std::vector<double> smoothAlphaVel;
    std::vector<double> smoothBetaVel;
    std::vector<double> scoreVec;


    vec3 basePos;
    vec3 iHat;
    vec3 jHat;
    vec3 kHat;
    vec3 dxyz;
    double alphaLen;
    //distance from alpha axis to end of beta collision segment
    double betaLen;
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

    std::vector<vec2> alphaPath, betaPath;
    std::vector<vec2> alphaPathBuffered, betaPathBuffered; // extended from alpha/betaPath in smoothing
    std::vector<vec2> roughAlphaX, roughAlphaY, roughBetaX, roughBetaY; // jiggly
    std::vector<vec2> smoothedAlphaPath, smoothedBetaPath;
    std::vector<vec2> simplifiedAlphaPath, simplifiedBetaPath; // sparse
    std::vector<vec2> interpSimplifiedAlphaPath, interpSimplifiedBetaPath; // dense
    std::vector<vec2> interpVelocityAlphaPath, interpVelocityBetaPath; // dense
    std::vector<vec2> interpAccelerationAlphaPath, interpAccelerationBetaPath; // dense
    std::vector<vec2> interpAlphaX, interpAlphaY, interpBetaX, interpBetaY; // smoothed
    std::vector<vec2> interpCollisions; // boolean points for collided or not
    std::vector<int> robotNeighbors; // robot IDs in RobotGrid.robotDict may potentially collide
    std::vector<int> fiducialNeighbors; // fiducial IDs in RobotGrid.fiducialDict may potentially collide
    std::vector<int> gfaNeighbors; // gfa IDs in RobotGrid.gfaDict may potentially collide
    std::vector<long> validTargetIDs; // target IDs in RobotGrid.targetDict that I can reach
    Robot (int id, std::string holeID, vec3 basePos, vec3 iHat, vec3 jHat,
            vec3 kHat, vec3 dxyz, double alphaLen, double alphaOffDeg,
            double betaOffDeg, double elementHeight, double scaleFac, vec2 metBetaXY,
            vec2 bossBetaXY, vec2 apBetaXY,
            std::array<vec2, 2> collisionSegBetaXY, double angStep = 1,
            bool hasApogee = true, double collisionBuffer = 2.0, bool lefthanded = false
    );
    void setAlphaBeta (double alpha, double beta);
    void setAlphaBetaFast (double alpha, double beta);
    void saveAlphaBeta();
    void setGreedPhobia (double greed, double phobia);
    void setDestinationAlphaBeta(double alpha, double beta);
    void setFiberToWokXYZ (vec3 wokXYZ, FiberType fiberType); // xy in focal plane coord sys

    double score(); // metric for how close to target I am

    void addRobotNeighbor(int robotID);
    void addFiducialNeighbor(int fiducialID);
    void addGFANeighbor(int fiducialID);

    void setXYUniform();
    bool isValidDither(vec2 newAlphaBeta);
    vec2 uniformDither(double radius);
    vec2 randomXYUniform();
    void simplifyPath(double epsilon);
    void smoothVelocity(int points);
    void setCollisionBuffer(double newBuffer);
    vec2 alphaBetaFromWokXYZ(vec3 wokXYZ, FiberType fiberType);
    vec2 convFiberXY(double x, double y, FiberType fromFiberType, FiberType toFiberType);
    void assignTarget(long targetID);
    void clearAssignment();
    double getMaxReach();
    bool isAssigned();

};
