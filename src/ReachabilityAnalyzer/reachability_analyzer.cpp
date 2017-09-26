#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"

#include "RemoteTreeViewerWrapper.hpp"

DEFINE_string(urdf,
              "drake/manipulation/models/iiwa_description/urdf/"
              "iiwa14_primitive_collision.urdf",
              "Path to robot URDF.");

DEFINE_string(end_effector_frame_name, "iiwa_frame_ee",
              "End effector frame name.");

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector3i;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Isometry3d;

using std::vector;

int DoMain(void) {
  auto model = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      drake::FindResourceOrThrow(FLAGS_urdf), drake::multibody::joints::kFixed,
      model.get());

  RemoteTreeViewerWrapper rm;

  // Provide a dummy timespan for our single-time-point
  // constraints.
  const Vector2d tspan(0, 1);

  // The first pose that we test will be all-zeros.
  std::default_random_engine e1(0);
  VectorXd q0 = model->getRandomConfiguration(e1);
  VectorXd qnom = model->getZeroConfiguration();

  // Data structures for storing the position constraint
  // for the end effector, which is updated every loop.
  Vector3d pos_end;
  // Vector4d quat_end;
  const double pos_tol = 0.01;
  Vector3d pos_lb, pos_ub;

  // Extract the end effector body_index and position
  // info from the supplied frame name.
  const auto end_effector_frame =
      model->findFrame(FLAGS_end_effector_frame_name);
  const int link_index = end_effector_frame->get_rigid_body().get_body_index();
  const Isometry3d frame_tf = end_effector_frame->get_transform_to_body();

  // Data structures that can be shared between ik calls.
  IKoptions ikoptions(model.get());
  //ikoptions.setMajorIterationsLimit(10);
  //ikoptions.setIterationsLimit(1000);

  printf("Running with major iter lim %d and iter limt %d\n", ikoptions.getMajorIterationsLimit(),
  ikoptions.getIterationsLimit());

  VectorXd q_sol = qnom;
  int info = 0;
  std::vector<std::string> infeasible_constraint;

  // Non-penetration constraint
  AllBodiesClosestDistanceConstraint nonpen(
      model.get(), 0.00, std::numeric_limits<double>::infinity(), {}, {});

  // Iteration information to define the volume.
  Vector3i steps(20, 20, 20);
  Vector3d min_val(-1.0, -1.0, -0.5);
  Vector3d max_val(1.0, 1.0, 1.5);

  // Store volume information as a colored point cloud.
  int n_pts = steps.prod();
  Matrix3Xd pts(3, n_pts);
  std::vector<std::vector<double>> colors(n_pts, std::vector<double>(3));

  int k_tried = 0;
  int k_reachable = 0;
  int last_published_k_reachable = k_reachable - 1;
  printf("\nasdf");
  for (int x_i = 0; x_i < steps[0]; x_i++) {
    for (int y_i = 0; y_i < steps[1]; y_i++) {
      for (int z_i = 0; z_i < steps[2]; z_i++) {
        Vector3d pos_end =
            min_val +
            ((Vector3d(x_i, y_i, z_i).cast<double>().array()) *
             (max_val - min_val).array() / steps.cast<double>().array())
                .matrix();

        pos_lb = pos_end - Vector3d::Constant(pos_tol);
        pos_ub = pos_end + Vector3d::Constant(pos_tol);

        WorldPositionConstraint wpc(model.get(), link_index,
                                    frame_tf.translation(), pos_lb, pos_ub,
                                    tspan);
        const std::vector<const RigidBodyConstraint*> constraint_array{&wpc};
                                                                      //&nonpen};

        // Important to randomly seed, because starting from the zero configuration
        // confuses SNOPT sometimes (due to bad gradients at that posture).
        q0 = model->getRandomConfiguration(e1);
        inverseKin(model.get(), q0, qnom, constraint_array.size(),
                   constraint_array.data(), ikoptions, &q_sol, &info,
                   &infeasible_constraint);

        if (info == 1) {
          pts.col(k_reachable) = pos_end;
          colors[k_reachable][0] = 0.0;
          colors[k_reachable][1] = 1.0;
          colors[k_reachable][2] = 0.0;
          k_reachable++;
        }

        k_tried++;
        printf("\rTried %d/%d, reachable %d/%d", k_tried, n_pts, k_reachable,
               n_pts);
        fflush(stdout);
        if (k_reachable % 100 == 0 &&
            k_reachable != last_published_k_reachable) {
          if (k_reachable > 0) {
            rm.publishPointCloud(pts.block(0, 0, 3, k_reachable),
                                 {"reachability", "pts"});
            rm.publishRigidBodyTree(*model, q_sol, Vector4d(0.5, 0.5, 0.5, 1.0),
                                  {"reachability", "robot"});
          }
          last_published_k_reachable = k_reachable;
        }
      }
    }
  }

  printf("\bTried %d/%d, reachable %d/%d\n", k_tried, n_pts, k_reachable, n_pts);
  if (k_reachable > 0) {
    rm.publishPointCloud(pts.block(0, 0, 3, k_reachable),
                         {"reachability", "pts"});
  }
  rm.publishRigidBodyTree(*model, q_sol, Vector4d(0.5, 0.5, 0.5, 1.0),
                          {"reachability", "robot"});
  last_published_k_reachable = k_reachable;

  return 0;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return DoMain();
}