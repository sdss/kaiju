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

    py::class_<GFA, std::shared_ptr<GFA>>(m, "GFA", py::dynamic_attr())
        .def_readwrite("collisionSegWokXYZ", &GFA::collisionSegWokXYZ)
        .def_readwrite("collisionBuffer", &GFA::collisionBuffer)
        .def_readwrite("id", &GFA::id);

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
                        std::array<vec2, 2>, double, bool, double>())
        .def_readwrite("alpha", &Robot::alpha, R"pbdoc(
            Robot's alpha position (degrees).
            )pbdoc")
        .def_readwrite("fiducialWatch", &Robot::fiducialWatch)
        .def_readwrite("greed", &Robot::greed)
        .def_readwrite("phobia", &Robot::phobia)
        .def_readwrite("isOffline", &Robot::isOffline)
        .def_readwrite("lefthanded", &Robot::lefthanded)
        .def_readwrite("robotNeighbors", &Robot::robotNeighbors)
        .def_readwrite("scoreVec", &Robot::scoreVec)
        .def_readwrite("fiducialNeighbors", &Robot::fiducialNeighbors)
        .def_readwrite("gfaNeighbors", &Robot::gfaNeighbors)
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
        .def_readwrite("iHat", &Robot::iHat)
        .def_readwrite("jHat", &Robot::jHat)
        .def_readwrite("kHat", &Robot::kHat)
        .def_readwrite("dxyz", &Robot::dxyz)
        .def_readwrite("scaleFac", &Robot::scaleFac)
        .def_readwrite("elementHeight", &Robot::elementHeight)
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
        .def_readwrite("alphaPathBuffered", &Robot::alphaPathBuffered)
        .def_readwrite("betaPathBuffered", &Robot::betaPathBuffered)
        .def_readwrite("smoothedAlphaPath", &Robot::smoothedAlphaPath)
        .def_readwrite("smoothedBetaPath", &Robot::smoothedBetaPath)
        .def_readwrite("simplifiedAlphaPath", &Robot::simplifiedAlphaPath)
        .def_readwrite("simplifiedBetaPath", &Robot::simplifiedBetaPath)
        .def_readwrite("interpSimplifiedAlphaPath", &Robot::interpSimplifiedAlphaPath)
        .def_readwrite("interpSimplifiedBetaPath", &Robot::interpSimplifiedBetaPath)
        .def_readwrite("interpVelocityAlphaPath", &Robot::interpVelocityAlphaPath)
        .def_readwrite("interpVelocityBetaPath", &Robot::interpVelocityBetaPath)
        .def_readwrite("interpAccelerationAlphaPath", &Robot::interpAccelerationAlphaPath)
        .def_readwrite("interpAccelerationBetaPath", &Robot::interpAccelerationBetaPath)
        .def_readwrite("interpAlphaX", &Robot::interpAlphaX)
        .def_readwrite("interpAlphaY", &Robot::interpAlphaY)
        .def_readwrite("interpBetaX", &Robot::interpBetaX)
        .def_readwrite("interpBetaY", &Robot::interpBetaY)
        .def_readwrite("roughAlphaX", &Robot::roughAlphaX)
        .def_readwrite("roughAlphaY", &Robot::roughAlphaY)
        .def_readwrite("roughBetaX", &Robot::roughBetaX)
        .def_readwrite("roughBetaY", &Robot::roughBetaY)
        .def_readwrite("interpCollisions", &Robot::interpCollisions)
        .def_readwrite("xPos", &Robot::xPos)
        .def_readwrite("yPos", &Robot::yPos)
        .def("setAlphaBeta", &Robot::setAlphaBeta, R"pbdoc(
            A doc example
        )pbdoc")
        .def("setDestinationAlphaBeta", &Robot::setDestinationAlphaBeta)
        .def("setXYUniform", &Robot::setXYUniform)
        .def("randomXYUniform", &Robot::randomXYUniform)
        .def("uniformDither", &Robot::uniformDither)
        .def("alphaBetaFromWokXYZ", &Robot::alphaBetaFromWokXYZ)
        .def("setFiberToWokXYZ", &Robot::setFiberToWokXYZ)
        .def("score", &Robot::score)
        .def("getMaxReach", &Robot::getMaxReach)
        .def("isAssigned", &Robot::isAssigned);



    py::class_<RobotGrid, std::shared_ptr<RobotGrid>>(m, "RobotGrid", py::dynamic_attr(), R"pbdoc(
            Robot Grid Class

            A class for holding a grid of robots.
        )pbdoc")
        .def(py::init<double, double, int>(),
            "angStep"_a=0.1, "epsilon"_a = 2, "seed"_a = 0)
        .def_readwrite("algType", &RobotGrid::algType)
        .def_readwrite("seed", &RobotGrid::seed)
        .def_readwrite("robotDict", &RobotGrid::robotDict)
        // .def_readwrite("greed", &RobotGrid::greed)
        // .def_readwrite("phobia", &RobotGrid::phobia)
        .def_readwrite("angStep", &RobotGrid::angStep)
        // .def_readwrite("collisionBuffer", &RobotGrid::collisionBuffer)
        .def_readwrite("smoothCollisions", &RobotGrid::smoothCollisions)
        .def_readwrite("smoothCollidedRobots", &RobotGrid::smoothCollidedRobots)
        .def_readwrite("didFail", &RobotGrid::didFail)
        .def_readwrite("nSteps", &RobotGrid::nSteps)
        .def_readwrite("mdp2iter", &RobotGrid::mdp2iter)
        // .def_readwrite("nStepsSmoothed", &RobotGrid::nStepsSmoothed)
        .def_readwrite("nRobots", &RobotGrid::nRobots)
        .def_readwrite("fiducialDict", &RobotGrid::fiducialDict)
        .def_readwrite("targetDict", &RobotGrid::targetDict)
        .def_readwrite("gfaDict", &RobotGrid::gfaDict)
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
               "hasApogee"_a = true, "collisionBuffer"_a=2.0,
               "lefthanded"_a = false)

        .def("addFiducial", &RobotGrid::addFiducial,
            "fiducialID"_a, "xyzWok"_a, "collisionBuffer"_a = 2.5)
        .def("addGFA", &RobotGrid::addGFA,
            "gfaID"_a, "collisionSegWokXYZ"_a, "collisionBuffer"_a = 2.5)
        .def("addTarget", &RobotGrid::addTarget,
            "targetID"_a, "xyzWok"_a, "fiberType"_a, "priority"_a = 0)

        .def("initGrid", &RobotGrid::initGrid)
        .def("decollideGrid", &RobotGrid::decollideGrid)
        .def("decollideRobot", &RobotGrid::decollideRobot)
        .def("homeRobot", &RobotGrid::homeRobot)
        .def("simplifyPaths", &RobotGrid::simplifyPaths)
        .def("smoothPaths", &RobotGrid::smoothPaths)
        .def("verifySmoothed", &RobotGrid::verifySmoothed)
        .def("setCollisionBuffer", &RobotGrid::setCollisionBuffer)
        .def("shrinkCollisionBuffer", &RobotGrid::shrinkCollisionBuffer)
        .def("growCollisionBuffer", &RobotGrid::growCollisionBuffer)
        .def("pathGenGreedy", &RobotGrid::pathGenGreedy, "stopIfDeadlock"_a = false, "ignoreInitialCollisions"_a = false)
        .def("pathGenMDP", &RobotGrid::pathGenMDP, "greed"_a, "phobia"_a, "ignoreInitialCollisions"_a = false)
        .def("pathGenMDP2", &RobotGrid::pathGenMDP2, "greed"_a, "phobia"_a, "ignoreInitialCollisions"_a = false, "nTries"_a = 5)
        .def("pathGenExplode", &RobotGrid::pathGenExplode)
        .def("pathGenExplodeOne", &RobotGrid::pathGenExplodeOne)
        .def("targetlessRobots", &RobotGrid::targetlessRobots)
        .def("unreachableTargets", &RobotGrid::unreachableTargets)
        .def("assignedTargets", &RobotGrid::assignedTargets)
        .def("getRobot", &RobotGrid::getRobot)
        .def("clearTargetDict", &RobotGrid::clearTargetDict)
        .def("unassignTarget", &RobotGrid::unassignTarget)
        .def("unassignRobot", &RobotGrid::unassignRobot)
        .def("isValidAssignment", &RobotGrid::isValidAssignment,
            "robotID"_a, "targID"_a)
        .def("assignRobot2Target", &RobotGrid::assignRobot2Target,
            "robotID"_a, "targID"_a)
        .def("unassignedRobots", &RobotGrid::unassignedRobots)
        .def("robotColliders", &RobotGrid::robotColliders)
        .def("fiducialColliders", &RobotGrid::fiducialColliders)
        .def("gfaColliders", &RobotGrid::gfaColliders)

        .def("getCollidedRobotRobotList", &RobotGrid::getCollidedRobotRobotList)
        .def("getCollidedRobotFiducialList", &RobotGrid::getCollidedRobotFiducialList)
        .def("getCollidedRobotGFAList", &RobotGrid::getCollidedRobotGFAList)
        .def("getCollidedRobotList", &RobotGrid::getCollidedRobotList)

        .def("isCollidedWithAssigned", &RobotGrid::isCollidedWithAssigned)
        .def("wouldCollideWithAssigned", &RobotGrid::wouldCollideWithAssigned)
        .def("isCollided", &RobotGrid::isCollided);
}
