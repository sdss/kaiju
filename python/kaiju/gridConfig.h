#pragma once
#include "robot.h"

const double beta_arm_width = 5.2;
const double buffer_distance = beta_arm_width / 2.0;
const double collide_dist_squared = beta_arm_width * beta_arm_width;
const double collide_dist_squared_shrink = 5 * 5; // collide zone just a bit
const double alpha_arm_len = 7.4;
const double beta_arm_len = 15; // mm to fiber
const double pitch = 22.4; // distance to next nearest neighbor
const double min_reach = beta_arm_len - alpha_arm_len;
const double max_reach = beta_arm_len + alpha_arm_len;

const double ang_step = 0.25; // degrees
const int maxPathStepsGlob = (int)(ceil(500.0/ang_step));
// line smoothing factor
const double epsilon =  5 * ang_step; // was 7*ang_step for 0.1 step size

class RobotGrid {
public:
    int nRobots;
    bool didFail;
    int nSteps;
    int maxPathSteps;
    int printEvery; // if -1 just print final if -2 print final and start

    double xFocalMax, yFocalMax, xFocalMin, yFocalMin;
    std::list<Robot> allRobots;
    RobotGrid (int nDia, int myMaxPathSteps);
    void decollide();
    int getNCollisions(double collide2);
    void toFile(const char*);
    void pathGen();
    void smoothPaths();
    void verifySmoothed();
};

bool robotSort(const Robot& robot1, const Robot& robot2);