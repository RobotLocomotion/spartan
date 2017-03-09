#include "RemoteTreeViewerWrapper.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;

static inline double randrange(double min, double max){
  return (((double)rand()) / RAND_MAX)*(max - min) + min;
}

static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

int main(int argc, char ** argv){
    // Visualize sim with the remote tree viewer
  RemoteTreeViewerWrapper rm;

  // Actual robot
  RigidBodyTree<double> tree;
  string shadow_hand_path;
  shadow_hand_path += getenv("SPARTAN_SOURCE_DIR");
  shadow_hand_path += "/models/shadow_hand/hand/shadow_hand_lite.urdf";
  AddModelInstanceFromUrdfFileWithRpyJointToWorld(shadow_hand_path, &tree);
  VectorXd q_robot = VectorXd::Zero(tree.get_num_positions());

  // print robot frame info for my personal reference
  for (int i=0; i < tree.get_num_bodies(); i++){
    cout << "Body " << i << ": \"" << 
      tree.getBodyOrFrameName(i) << "\"" << endl;
  }

  struct GoalInfo {
    Vector3d world_goal;
    int goal_frame;
    Vector3d body_point;
    double weight;
  };
  vector<GoalInfo> goals;

  goals.push_back(
    GoalInfo({Vector3d(0.0, 0.0, 0.4), 
              tree.FindBodyIndex("rh_fftip"), 
              Vector3d(0., 0., 0.), 
              500.0})
  );
  goals.push_back(
    GoalInfo({Vector3d(0.0, 0.0, 0.4), 
              tree.FindBodyIndex("rh_mftip"), 
              Vector3d(0., 0., 0.), 
              500.0})
  );
  goals.push_back(
    GoalInfo({Vector3d(0.0, 0.0, 0.4), 
              tree.FindBodyIndex("rh_rftip"), 
              Vector3d(0., 0., 0.), 
              500.0})
  );
  goals.push_back(
    GoalInfo({Vector3d(0.0, 0.0, 0.4), 
              tree.FindBodyIndex("rh_thtip"), 
              Vector3d(0., 0., 0.), 
              500.0})
  );

  const double dt = 0.0333;
  double t = 0.0;
  double start_time = getUnixTime();


  // Kick off vis
  Affine3d tf_robot;
  tf_robot.setIdentity();
  rm.publishRigidBodyTree(tree, q_robot, Vector4d(0.3, 0.3, 1.0, 1.0), {"shadow_hand_sim_gt"});

  while (t < 10.0){
    double t_actual = getUnixTime();
    if (t_actual - start_time < t){
      // slow us down to real time
      usleep(1000*1000*(t - (t_actual - start_time)));
    }


    KinematicsCache<double> robot_kinematics_cache = tree.doKinematics(q_robot);
    for (auto& goal : goals){
      Vector3d body_point_in_world = 
        tree.transformPoints<double>(robot_kinematics_cache, 
                    goal.body_point, goal.goal_frame, 0);
      MatrixXd body_point_jacobian = 
        tree.transformPointsJacobian<double>(robot_kinematics_cache, 
                    goal.body_point, goal.goal_frame, 0, false);
      Vector3d error = goal.world_goal - body_point_in_world;
      VectorXd correction = body_point_jacobian.transpose() * error;
      q_robot += correction * dt * goal.weight;
    }
    cout << "q robot " << q_robot.transpose() << endl;

    rm.updateRigidBodyTree(tree, q_robot, {"shadow_hand_sim_gt"});
    t += dt;
  }



  return 0;
}
