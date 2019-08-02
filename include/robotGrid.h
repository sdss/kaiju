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
    std::vector<std::shared_ptr<Robot>> allRobots;
    std::vector<std::array<double, 2>> fiducialList;
    std::vector<std::shared_ptr<Target>> targetList;
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
    void setTargetList(Eigen::MatrixXd myTargetList); //std::vector<std::array<double, 5>> myTargetList);
    std::shared_ptr<Robot> getRobot(int);
    std::vector<std::shared_ptr<Robot>> targetlessRobots();
    std::vector<std::shared_ptr<Target>> unreachableTargets();
    std::vector<std::shared_ptr<Target>> assignedTargets();
    void pairwiseSwap();
    void swapTargets(std::shared_ptr<Robot> r1, std::shared_ptr<Robot> r2);
    void greedyAssign();
    void clearTargetList();
    void assignRobot2Target(int robotInd, int targID);
    std::vector<std::shared_ptr<Robot>> unassignedRobots();
};
