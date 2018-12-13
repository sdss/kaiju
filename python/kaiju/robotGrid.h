#pragma once
#include "robot.h"

// move constants to cpp file?
extern const double beta_arm_width;
extern const double buffer_distance;
extern const double collide_dist_squared;
extern const double collide_dist_squared_shrink;
extern const double alpha_arm_len;
extern const double beta_arm_len;
extern const double pitch;
extern const double min_reach;
extern const double max_reach;

extern const double radius_buffer;

extern const double ang_step;
extern const int maxPathStepsGlob;
// line smoothing factor
extern const double epsilon;
extern const double min_targ_sep;

class RobotGrid {
public:
    int nRobots;
    bool didFail;
    int nSteps;
    int maxPathSteps;
    int printEvery; // if -1 just print final if -2 print final and start
    double xFocalMax, yFocalMax, xFocalMin, yFocalMin;
    std::vector<Robot> allRobots;
    RobotGrid (int nDia, int myMaxPathSteps, int myPrintEvery = 0);
    void decollide();
    int getNCollisions(double radiusBuffer=0);
    void toFile(const char*);
    void printStats(const char*);
    void pathGen();
    void smoothPaths();
    void verifySmoothed();
    void optimizeTargets();
};
