#include <pybind11/pybind11.h>
//#include <pybind11/eigen.h>
//#include <pybind11/stl.h>

#include "rescale_trajectory.hpp"

namespace py = pybind11;

PYBIND11_PLUGIN(py_rescale) {
    py::module m("py_rescale", "Rescale kinematic trajectory");

    m.def("RescaleTrajectory", &scaleTime);

    return m.ptr();
}
