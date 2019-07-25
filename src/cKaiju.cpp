#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
// #include <pybind11/stl_bind.h>
#include "robotGrid.h"
#include "target.h"

// PYBIND11_MAKE_OPAQUE(std::vector<Robot>);

namespace py = pybind11;

PYBIND11_MODULE(cKaiju, m) {

    // m.def("initBetaArms", &initBetaArms, R"pbdoc(
    //     Initialize Beta arms

    //     This routine must be run before any path planning.

    // )pbdoc");

    // m.def("getBetaGeom", &getBetaGeom);
    py::class_<Target, std::shared_ptr<Target>>(m, "Target")
        .def_readwrite("x", &Target::x)
        .def_readwrite("y", &Target::y)
        .def_readwrite("validRobots", &Target::validRobots);

    py::class_<Robot, std::shared_ptr<Robot>>(m, "Robot", R"pbdoc(
        A robot positioner class

        This class is something totally awesome.

        )pbdoc")
        .def(py::init<int, double, double, double, bool>())
        .def_readwrite("alpha", &Robot::alpha, R"pbdoc(
            Robot's alpha position (degrees).
            )pbdoc")
        .def_readwrite("beta", &Robot::beta)
        .def_readwrite("xPos", &Robot::xPos)
        .def_readwrite("yPos", &Robot::yPos)
        .def_readwrite("metFiberPos", &Robot::metFiberPos)
        .def_readwrite("bossFiberPos", &Robot::bossFiberPos)
        .def_readwrite("apFiberPos", &Robot::apFiberPos)
        .def_readwrite("nDecollide", &Robot::nDecollide)
        .def_readwrite("betaCollisionSegment", &Robot::betaCollisionSegment)
        // .def_readwrite("betaOrientation", &Robot::betaOrientation)
        // .def_readwrite("betaModel", &Robot::betaModel)
        .def_readwrite("id", &Robot::id)
        // .def_readwrite("lastStepNum", &Robot::lastStepNum)
        .def_readwrite("alphaPath", &Robot::alphaPath)
        .def_readwrite("betaPath", &Robot::betaPath)
        .def_readwrite("smoothAlphaPath", &Robot::smoothAlphaPath)
        .def_readwrite("smoothBetaPath", &Robot::smoothBetaPath)
        .def_readwrite("interpSmoothAlphaPath", &Robot::interpSmoothAlphaPath)
        .def_readwrite("interpSmoothBetaPath", &Robot::interpSmoothBetaPath)
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
            Set alpha beta

            And some other bs.
        )pbdoc")
        .def("setXYUniform", &Robot::setXYUniform)
        .def("alphaBetaFromFiberXY", &Robot::alphaBetaFromFiberXY)
        .def("setAlphaBetaRand", &Robot::setAlphaBetaRand)
        .def("isCollided", &Robot::isCollided)
        // .def("checkFiberXYLocal", &Robot::checkFiberXYLocal)
        // .def("checkFiberXYGlobal", &Robot::checkFiberXYGlobal)
        .def("setFiberXY", &Robot::setFiberXY)
        .def("decollide", &Robot::decollide);

    py::class_<RobotGrid, std::shared_ptr<RobotGrid>>(m, "RobotGrid", R"pbdoc(
            Robot Grid Class

            A class for holding a grid of robots.
        )pbdoc")
        .def(py::init<double, double, double, int>())
        // warning!!! iterating over this gives you copies!!!! use getRobot if you want a reference
        .def_readwrite("allRobots", &RobotGrid::allRobots)
        .def_readwrite("targetList", &RobotGrid::targetList)
        .def_readwrite("smoothCollisions", &RobotGrid::smoothCollisions)
        .def_readwrite("didFail", &RobotGrid::didFail)
        .def_readwrite("nSteps", &RobotGrid::nSteps)
        .def_readwrite("nRobots", &RobotGrid::nRobots)
        .def_readwrite("fiducialList", &RobotGrid::fiducialList)
        .def_readwrite("targetList", &RobotGrid::targetList)
        .def("addRobot", &RobotGrid::addRobot)
        .def("addFiducial", &RobotGrid::addFiducial)
        .def("initGrid", &RobotGrid::initGrid)
        .def("optimizeTargets", &RobotGrid::optimizeTargets)
        .def("decollide", &RobotGrid::decollide)
        .def("smoothPaths", &RobotGrid::smoothPaths)
        .def("verifySmoothed", &RobotGrid::verifySmoothed)
        .def("setCollisionBuffer", &RobotGrid::setCollisionBuffer)
        .def("pathGen", &RobotGrid::pathGen)
        .def("setTargetList", &RobotGrid::setTargetList)
        .def("getRobot", &RobotGrid::getRobot);
}

