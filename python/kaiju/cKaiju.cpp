#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "robotGrid.h"
#include "betaArm.h"

namespace py = pybind11;

PYBIND11_MODULE(cKaiju, m) {
    m.def("initBetaArms", &initBetaArms);
    m.def("getBetaGeom", &getBetaGeom);
    py::class_<Robot>(m, "Robot")
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
        .def_readwrite("interpCollisions", &Robot::interpCollisions);

    py::class_<RobotGrid>(m, "RobotGrid")
        .def(py::init<int, double, int, int, double>())
        .def_readwrite("allRobots", &RobotGrid::allRobots)
        .def("optimizeTargets", &RobotGrid::optimizeTargets)
        .def("decollide", &RobotGrid::decollide)
        .def("smoothPaths", &RobotGrid::smoothPaths)
        .def("verifySmoothed", &RobotGrid::verifySmoothed)
        .def("setCollisionBuffer", &RobotGrid::setCollisionBuffer)
        .def("pathGen", &RobotGrid::pathGen);
}

