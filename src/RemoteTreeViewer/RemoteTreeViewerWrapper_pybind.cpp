#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "RemoteTreeViewerWrapper.hpp"

namespace py = pybind11;

PYBIND11_MODULE(RemoteTreeViewerWrapper_pybind, m) {
  py::module::import("pydrake.rbtree");

  m.doc() = "Utilities for using the Director RemoteTreeViewer conveniently.";

  py::class_<RemoteTreeViewerWrapper>(m, "RemoteTreeViewerWrapper")
      .def(py::init<>())
      .def("publishPointCloud", &RemoteTreeViewerWrapper::publishPointCloud)
      .def("publishLine", &RemoteTreeViewerWrapper::publishLine)
      .def("publishRawMesh", &RemoteTreeViewerWrapper::publishRawMesh)
      .def("publishRigidBodyTree",
           &RemoteTreeViewerWrapper::publishRigidBodyTree)
      .def("publishRigidBody", &RemoteTreeViewerWrapper::publishRigidBody)
      .def("publishGeometry", &RemoteTreeViewerWrapper::publishGeometry);
  m.attr("__version__") = "dev";
}