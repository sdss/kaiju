#include <pybind11/pybind11.h>
#include "robotGrid.h"
#include "betaArm.h"

namespace py = pybind11;

PYBIND11_MODULE(cKaiju, m) {
    m.def("initBetaArms", &initBetaArms);
    m.def("setBetaGeom", &setBetaGeom);
    py::class_<RobotGrid>(m, "RobotGrid")
        .def(py::init<int, double, int, double>())
        .def("optimizeTargets", &RobotGrid::optimizeTargets)
        .def("decollide", &RobotGrid::decollide)
        .def("smoothPaths", &RobotGrid::smoothPaths)
        .def("verifySmoothed", &RobotGrid::verifySmoothed)
        .def("setCollisionBuffer", &RobotGrid::setCollisionBuffer)
        .def("pathGen", &RobotGrid::pathGen);
}

