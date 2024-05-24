#pragma once
#include <set>
#include "robot.h"
#include "target.h"
#include "fiducial.h"
#include "gfa.h"


enum AlgType {Greedy, MDP, Fold}; // order is important

vec2 handleLimits(double currAlpha, double currBeta, double nextAlpha, double nextBeta);

class RobotGrid {
public:
    AlgType algType;
    int nRobots;
    int mdp2iter; // number of iterations used in mdp2 routine
    double epsilon;
    // double collisionBuffer;
    double angStep;
    // double greed;
    // double phobia;
    bool didFail;
    int nSteps = 0;
    int seed;
    int maxPathSteps;
    int smoothCollisions;
    std::set<int> smoothCollidedRobots;

    bool initialized = false;
    double maxDisplacement;
    std::map<int, std::shared_ptr<Robot>> robotDict;
    std::map<int, std::shared_ptr<Fiducial>> fiducialDict;
    std::map<int, std::shared_ptr<GFA>> gfaDict;
    std::map<long, std::shared_ptr<Target>> targetDict;
    std::vector<vec2> perturbArray; // alpha/beta perturbations

    RobotGrid (double angStep = 1, double epsilon = 2, int seed = 0);
    void addRobot(
        int robotID, std::string holeID, vec3 basePos, vec3 iHat, vec3 jHat,
        vec3 kHat, vec3 dxyz, double alphaLen, double alphaOffDeg,
        double betaOffDeg, double elementHeight, double scaleFac, vec2 metBetaXY,
        vec2 bossBetaXY, vec2 apBetaXY,
        std::array<vec2, 2> collisionSegBetaXY, bool hasApogee = true, double collisionBuffer = 2.0,
        bool lefthanded = false
    );
    void addTarget(long targetID, vec3 xyzWok, FiberType fiberType, double priority = 0);
    void addFiducial(int fiducialID, vec3 xyzWok, double collisionBuffer = 2.5);
    void addGFA(int gfaID, std::array<vec3, 2> collisionSegWokXYZ, double collisionBuffer = 2.5);
    void initGrid();
    void decollideGrid();
    int getNCollisions();
    std::vector<int> deadlockedRobots(); // robots not on target
    void clearPaths();

    void pathGenGreedy(bool stopIfDeadlock, bool ignoreInitialCollisions); // stepRotational with encroachment
    void pathGenMDP(double greed, double phobia, bool ignoreInitialCollisions); // Markov Decision Process
    void pathGenMDP2(double greed, double phobia, bool ignoreInitialCollisions, int nTries); // Markov Decision Process
    void simplifyPaths();
    void smoothPaths(int points);
    void verifySmoothed(int totalSteps);

    // only modifies robot collision buffers
    void setCollisionBuffer(double newBuffer);
    void shrinkCollisionBuffer(double absShrink);
    void growCollisionBuffer(double absGrow);

    std::shared_ptr<Robot> getRobot(int robotID);
    std::vector<int> targetlessRobots(); // returns robotIDs
    std::vector<long> unreachableTargets(); // returns targetIDs
    std::vector<long> assignedTargets(); // returns targetIDs
    bool throwAway(int robotID);
    bool replaceNearFold(int robotID);

    void clearTargetDict();
    void assignRobot2Target(int robotID, long targID);
    void unassignTarget(long targID);
    void unassignRobot(int robotID);
    bool isValidAssignment(int robotID, long targID);
    std::vector<int> unassignedRobots();

    bool isCollided(int robotID);
    std::tuple<bool, bool, bool, std::vector<int>> isCollidedWithAssigned(int robotID);
    std::tuple<bool, bool, bool, std::vector<int>> wouldCollideWithAssigned(int robotID, long targID);
    std::vector<int> robotColliders(int robotID);
    std::vector<int> fiducialColliders(int robotID);
    std::vector<int> gfaColliders(int robotID);

    std::vector<int> getCollidedRobotRobotList();
    std::vector<int> getCollidedRobotFiducialList();
    std::vector<int> getCollidedRobotGFAList();
    std::vector<int> getCollidedRobotList();

    bool neighborEncroachment(std::shared_ptr<Robot> r1, double encroachDist);

    void decollideRobot(int robotID);
    void homeRobot(int robotID);
    void stepTowardFold(std::shared_ptr<Robot> r1, int stepNum);

    void stepGreedy(std::shared_ptr<Robot> r1, int stepNum);
    void stepMDP(std::shared_ptr<Robot> r1, int stepNum);
    void stepMDP2(std::shared_ptr<Robot> r1, int stepNum);
    void stepBeta(std::shared_ptr<Robot> r1, int stepNum);
    void stepDecollide(std::shared_ptr<Robot>, int stepNum);
    double minCollideDist(int robotID);
    void pathGenExplode(double deg2move);
    void pathGenExplodeOne(double deg2move, int robotID);

};
