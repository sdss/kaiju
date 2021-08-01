#include <pybind11/pybind11.h>
// #include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "robotGrid.h"
#include "target.h"
#include "fiducial.h"
// #include "coordio.h"


namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(cKaiju, m) {
    py::enum_<FiberType>(m, "FiberType", py::arithmetic())
        .value("MetrologyFiber", MetrologyFiber)
        .value("ApogeeFiber", ApogeeFiber)
        .value("BossFiber", BossFiber)
        .export_values();

    py::enum_<AlgType>(m, "AlgType", py::arithmetic())
        .value("Greedy", Greedy)
        .value("MDP", MDP)
        .value("Fold", Fold)
        .export_values();

    py::class_<Fiducial, std::shared_ptr<Fiducial>>(m, "Fiducial", py::dynamic_attr())
        .def_readwrite("xWok", &Fiducial::x)
        .def_readwrite("yWok", &Fiducial::y)
        .def_readwrite("xyzWok", &Fiducial::xyzWok)
        .def_readwrite("collisionBuffer", &Fiducial::collisionBuffer)
        .def_readwrite("id", &Fiducial::id);

    py::class_<Target, std::shared_ptr<Target>>(m, "Target", py::dynamic_attr())
        .def_readwrite("xWok", &Target::x)
        .def_readwrite("yWok", &Target::y)
        .def_readwrite("zWok", &Target::z)
        .def_readwrite("xyzWok", &Target::xyzWok)
        .def_readwrite("id", &Target::id)
        .def_readwrite("priority", &Target::priority)
        .def_readwrite("fiberType", &Target::fiberType)
        .def_readwrite("validRobotIDs", &Target::validRobotIDs)
        .def("isAssigned", &Target::isAssigned);

    py::class_<Robot, std::shared_ptr<Robot>>(m, "Robot", py::dynamic_attr(), R"pbdoc(
        A robot positioner class
        )pbdoc")
        .def(py::init<int, std::string, vec3, vec3, vec3, vec3,
                        vec3, double, double, double,
                        double, double, vec2, vec2, vec2,
                        std::array<vec2, 2>, double, bool>())
        .def_readwrite("alpha", &Robot::alpha, R"pbdoc(
            Robot's alpha position (degrees).
            )pbdoc")
        .def_readwrite("isOffline", &Robot::isOffline)
        .def_readwrite("robotNeighbors", &Robot::robotNeighbors)
        .def_readwrite("scoreVec", &Robot::scoreVec)
        .def_readwrite("fiducialNeighbors", &Robot::fiducialNeighbors)
        .def_readwrite("angStep", &Robot::angStep)
        .def_readwrite("collisionBuffer", &Robot::collisionBuffer)
        .def_readwrite("lastStepNum", &Robot::lastStepNum)
        .def_readwrite("destinationAlpha", &Robot::destinationAlpha)
        .def_readwrite("destinationBeta", &Robot::destinationBeta)
        .def_readwrite("alphaVel", &Robot::alphaVel)
        .def_readwrite("betaVel", &Robot::betaVel)
        .def_readwrite("smoothAlphaVel", &Robot::smoothAlphaVel)
        .def_readwrite("smoothBetaVel", &Robot::smoothBetaVel)
        .def_readwrite("beta", &Robot::beta)
        .def_readwrite("alphaOffDeg", &Robot::alphaOffDeg)
        .def_readwrite("betaOffDeg", &Robot::betaOffDeg)
        .def_readwrite("basePos", &Robot::basePos)
        .def_readwrite("validTargetIDs", &Robot::validTargetIDs)
        .def_readwrite("robotNeighbors", &Robot::robotNeighbors)
        .def_readwrite("hasApogee", &Robot::hasApogee)
        .def_readwrite("hasBoss", &Robot::hasBoss)
        .def_readwrite("metWokXYZ", &Robot::metWokXYZ)
        .def_readwrite("bossWokXYZ", &Robot::bossWokXYZ)
        .def_readwrite("apWokXYZ", &Robot::apWokXYZ)
        .def_readwrite("nDecollide", &Robot::nDecollide)
        .def_readwrite("collisionSegWokXYZ", &Robot::collisionSegWokXYZ)
        .def_readwrite("id", &Robot::id)
        .def_readwrite("holeID", &Robot::holeID)
        .def_readwrite("assignedTargetID", &Robot::assignedTargetID)
        .def_readwrite("alphaPath", &Robot::alphaPath)
        .def_readwrite("betaPath", &Robot::betaPath)
        // .def_readwrite("onTargetVec", &Robot::onTargetVec)
        .def_readwrite("smoothedAlphaPath", &Robot::smoothedAlphaPath)
        .def_readwrite("smoothedBetaPath", &Robot::smoothedBetaPath)
        .def_readwrite("simplifiedAlphaPath", &Robot::simplifiedAlphaPath)
        .def_readwrite("simplifiedBetaPath", &Robot::simplifiedBetaPath)
        .def_readwrite("interpSimplifiedAlphaPath", &Robot::interpSimplifiedAlphaPath)
        .def_readwrite("interpSimplifiedBetaPath", &Robot::interpSimplifiedBetaPath)
        .def_readwrite("interpAlphaX", &Robot::interpAlphaX)
        .def_readwrite("interpAlphaY", &Robot::interpAlphaY)
        .def_readwrite("interpBetaX", &Robot::interpBetaX)
        .def_readwrite("interpBetaY", &Robot::interpBetaY)
        .def_readwrite("roughAlphaX", &Robot::roughAlphaX)
        .def_readwrite("roughAlphaY", &Robot::roughAlphaY)
        .def_readwrite("roughBetaX", &Robot::roughBetaX)
        .def_readwrite("roughBetaY", &Robot::roughBetaY)
        .def_readwrite("interpCollisions", &Robot::interpCollisions)
        .def("setAlphaBeta", &Robot::setAlphaBeta, R"pbdoc(
            A doc example
        )pbdoc")
        .def("setDestinationAlphaBeta", &Robot::setDestinationAlphaBeta)
        .def("setXYUniform", &Robot::setXYUniform)
        .def("randomXYUniform", &Robot::randomXYUniform)
        .def("alphaBetaFromWokXYZ", &Robot::alphaBetaFromWokXYZ)
        // .def("setAlphaBetaRand", &Robot::setAlphaBetaRand)
        // .def("isCollided", &Robot::isCollided)
        .def("setFiberToWokXYZ", &Robot::setFiberToWokXYZ)
        .def("score", &Robot::score)
        // .def("decollide", &Robot::decollide)
        .def("getMaxReach", &Robot::getMaxReach)
        .def("isAssigned", &Robot::isAssigned);

    py::class_<RobotGrid, std::shared_ptr<RobotGrid>>(m, "RobotGrid", py::dynamic_attr(), R"pbdoc(
            Robot Grid Class

            A class for holding a grid of robots.
        )pbdoc")
        .def(py::init<double, double, double, int>(),
            "angStep"_a=1, "collisionBuffer"_a = 2, "epsilon"_a = 2, "seed"_a = 0)
        .def_readwrite("algType", &RobotGrid::algType)
        .def_readwrite("seed", &RobotGrid::seed)
        .def_readwrite("robotDict", &RobotGrid::robotDict)
        .def_readwrite("greed", &RobotGrid::greed)
        .def_readwrite("phobia", &RobotGrid::phobia)
        .def_readwrite("angStep", &RobotGrid::angStep)
        .def_readwrite("collisionBuffer", &RobotGrid::collisionBuffer)
        .def_readwrite("smoothCollisions", &RobotGrid::smoothCollisions)
        .def_readwrite("didFail", &RobotGrid::didFail)
        .def_readwrite("nSteps", &RobotGrid::nSteps)
        .def_readwrite("nRobots", &RobotGrid::nRobots)
        .def_readwrite("fiducialDict", &RobotGrid::fiducialDict)
        .def_readwrite("targetDict", &RobotGrid::targetDict)
        .def_readwrite("maxPathSteps", &RobotGrid::maxPathSteps)
        .def_readwrite("maxDisplacement", &RobotGrid::maxDisplacement)
        .def("throwAway", &RobotGrid::throwAway)
        .def("getNCollisions", &RobotGrid::getNCollisions)
        .def("deadlockedRobots", &RobotGrid::deadlockedRobots)
        .def("addRobot", &RobotGrid::addRobot,
                "robotID"_a, "holeID"_a, "basePos"_a, "iHat"_a, "jHat"_a,
                "kHat"_a, "dxyz"_a, "alphaLen"_a, "alphaOffDeg"_a,
                "betaOffDeg"_a, "elementHeight"_a, "scaleFac"_a, "metBetaXY"_a,
                "bossBetaXY"_a, "apBetaXY"_a,
                "collisionSegBetaXY"_a,
               "hasApogee"_a = true)
        // .def("addRobot", &RobotGrid::addRobot,
        //     "robotID"_a, "holeID"_a, "xPos"_a, "yPos"_a, "hasApogee"_a = true)
        .def("addFiducial", &RobotGrid::addFiducial,
            "fiducialID"_a, "xyzWok"_a, "collisionBuffer"_a = 1.5)
        .def("addTarget", &RobotGrid::addTarget,
            "targetID"_a, "xyzWok"_a, "fiberType"_a, "priority"_a = 0)
        // .def("addTarget", &RobotGrid::addTarget,
        //     "targetID"_a, "x"_a, "y"_a, "fiberType"_a, "priority"_a = 0)
        .def("initGrid", &RobotGrid::initGrid)
        // .def("optimizeTargets", &RobotGrid::optimizeTargets)
        .def("decollideGrid", &RobotGrid::decollideGrid)
        .def("decollideRobot", &RobotGrid::decollideRobot)
        .def("homeRobot", &RobotGrid::homeRobot)
        .def("simplifyPaths", &RobotGrid::simplifyPaths)
        .def("smoothPaths", &RobotGrid::smoothPaths)
        .def("verifySmoothed", &RobotGrid::verifySmoothed)
        .def("setCollisionBuffer", &RobotGrid::setCollisionBuffer)
        // .def("pathGen", &RobotGrid::pathGen)
        .def("pathGenGreedy", &RobotGrid::pathGenGreedy)
        .def("pathGenMDP", &RobotGrid::pathGenMDP)
        .def("pathGenEscape", &RobotGrid::pathGenEscape)
        // .def("setTargetList", &RobotGrid::setTargetList)
        // .def("addTargetList", &RobotGrid::addTargetList)
        .def("targetlessRobots", &RobotGrid::targetlessRobots)
        .def("unreachableTargets", &RobotGrid::unreachableTargets)
        .def("assignedTargets", &RobotGrid::assignedTargets)
        .def("getRobot", &RobotGrid::getRobot)
        .def("clearTargetDict", &RobotGrid::clearTargetDict)
        // .def("isValidRobotTarget", &RobotGrid::isValidRobotTarget)
        .def("unassignTarget", &RobotGrid::unassignTarget)
        .def("unassignRobot", &RobotGrid::unassignRobot)
        .def("isValidAssignment", &RobotGrid::isValidAssignment,
            "robotID"_a, "targID"_a)
        .def("assignRobot2Target", &RobotGrid::assignRobot2Target,
            "robotID"_a, "targID"_a)
        // .def("pairwiseSwap", &RobotGrid::pairwiseSwap)
        .def("unassignedRobots", &RobotGrid::unassignedRobots)
        .def("robotColliders", &RobotGrid::robotColliders)
        .def("fiducialColliders", &RobotGrid::fiducialColliders)
        .def("isCollidedWithAssigned", &RobotGrid::isCollidedWithAssigned)
        .def("wouldCollideWithAssigned", &RobotGrid::wouldCollideWithAssigned)
        .def("isCollided", &RobotGrid::isCollided);
}

