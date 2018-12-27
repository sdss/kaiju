#pragma once
#include "robot.h"

// move constants to cpp file?
extern const double alpha_arm_len;
extern const double beta_arm_len;
extern const double pitch;
extern const double min_reach;
extern const double max_reach;

extern const double radius_buffer;

// extern const double ang_step;
// extern const int maxPathStepsGlob;
// line smoothing factor
// extern const double epsilon;
extern const double min_targ_sep;

class RobotGrid {
public:
    double ang_step;
    double epsilon;
    int nRobots;
    bool didFail;
    int nSteps;
    int maxPathSteps;
    int printEvery; // if -1 just print final if -2 print final and start
    double xFocalMax, yFocalMax, xFocalMin, yFocalMin;
    std::vector<Robot> allRobots;
    RobotGrid (int nDia, double myAng_step, int betaGeomID = 6, int myPrintEvery = 0, double collisionBuffer=0);
    void decollide();
    int getNCollisions();
    void toFile(const char*);
    void printStats(const char*);
    void pathGen();
    void smoothPaths();
    void verifySmoothed();
    void optimizeTargets();
    void setCollisionBuffer(double newBuffer);
};
