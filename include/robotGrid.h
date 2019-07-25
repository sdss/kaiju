#pragma once
#include "robot.h"
#include "target.h"
// #include <pybind11/stl_bind.h>

// move constants to cpp file?
extern const double pitch;

extern const double radius_buffer;

// extern const double ang_step;
// extern const int maxPathStepsGlob;
// line smoothing factor
// extern const double epsilon;
extern const double min_targ_sep;

class RobotGrid {
public:
    int nRobots;
    double epsilon;
    double collisionBuffer;
    double angStep;
    bool didFail;
    int nSteps;
    int maxPathSteps;
    int smoothCollisions;
    // double xFocalMax, yFocalMax, xFocalMin, yFocalMin;
    std::vector<std::shared_ptr<Robot>> allRobots;
    std::vector<std::array<double, 2>> fiducialList;
    std::vector<Target> targetList;
    // std::vector<std::vector<int>> target2RobotMap, robot2TargetMap;
    RobotGrid (double myAngStep, double myCollisionBuffer, double myEpsilon, int seed);
    void addRobot(int robotID, double xPos, double yPos, bool hasApogee);
    void addFiducial(double xPos, double yPos);
    void initGrid();
    void decollide();
    int getNCollisions();
    void pathGen();
    void smoothPaths();
    void verifySmoothed();
    void optimizeTargets();
    void setCollisionBuffer(double newBuffer);
    void setTargetList(std::vector<std::array<double, 5>> myTargetList);
    std::shared_ptr<Robot> getRobot(int);
};
