#include "RemoteTreeViewerWrapper.hpp"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/util/lcmUtil.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "lcmtypes/drake/lcmt_body_motion_data.hpp"

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;
using namespace drake::lcm;

static inline double randrange(double min, double max){
  return (((double)rand()) / RAND_MAX)*(max - min) + min;
}

static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

class RequestedControlPointMonitor : public DrakeLcmMessageHandlerInterface {
 public:
  const RigidBodyTree<double> * tree_;
  struct GoalInfo {
    PiecewisePolynomial<double> world_goal;
    int goal_frame;
    Vector3d body_point;
    double weight;
  };
  vector<GoalInfo> goals;


  RequestedControlPointMonitor(const RigidBodyTree<double> * tree) : 
    tree_(tree) {};
  
  void HandleMessage(const std::string& channel,
                     const void* message_buffer, 
                     int message_size) {
    drake::lcmt_body_motion_data msg;
    msg.decode(message_buffer, 0, message_size);

    // Compact this info into a GoalInfo struct
    GoalInfo new_goal;
    new_goal.world_goal = decodePiecewisePolynomial(msg.spline);
    new_goal.goal_frame = tree_->FindBodyIndex(msg.body_or_frame_name);
    new_goal.body_point = Vector3d::Zero();
    new_goal.weight = msg.weight_multiplier[0];

    // Either update an old goal or add this new one
    bool found_goal = false;
    for (auto & goal : goals){
      if (goal.goal_frame = new_goal.goal_frame){
        goal = new_goal;
        found_goal = true;
        break;
      }
    }
    if (!found_goal){
      goals.push_back(new_goal);
    }
  };
};

int main(int argc, char ** argv){
  // Actual robot
  RigidBodyTree<double> tree;
  string shadow_hand_path;
  shadow_hand_path += getenv("SPARTAN_SOURCE_DIR");
  shadow_hand_path += "/models/shadow_hand/hand/shadow_hand_lite.urdf";
  AddModelInstanceFromUrdfFileWithRpyJointToWorld(shadow_hand_path, &tree);
  VectorXd q_robot = VectorXd::Zero(tree.get_num_positions());


  // Our LCM interface
  DrakeLcm drakeLCM;
  RequestedControlPointMonitor controlPointMonitor(&tree);
  drakeLCM.Subscribe("SH_GOALS", &controlPointMonitor);
  drakeLCM.StartReceiveThread();
  // Visualize sim with the remote tree viewer
  RemoteTreeViewerWrapper rm;

  // print robot frame info for my personal reference
  for (int i=0; i < tree.get_num_bodies(); i++){
    cout << "Body " << i << ": \"" << 
      tree.getBodyOrFrameName(i) << "\"" << endl;
  }

  const double dt = 0.0333;
  double t = 0.0;
  double start_time = getUnixTime();


  // Kick off vis
  Affine3d tf_robot;
  tf_robot.setIdentity();
  rm.publishRigidBodyTree(tree, q_robot, Vector4d(0.3, 0.3, 1.0, 1.0), {"shadow_hand_sim_gt"});

  while (1){
    double t_actual = getUnixTime();
    if (t_actual - start_time < t){
      // slow us down to real time
      usleep(1000*1000*(t - (t_actual - start_time)));
    }


    KinematicsCache<double> robot_kinematics_cache = tree.doKinematics(q_robot);
    for (auto& goal : controlPointMonitor.goals){
      Vector3d body_point_in_world = 
        tree.transformPoints<double>(robot_kinematics_cache, 
                    goal.body_point, goal.goal_frame, 0);
      MatrixXd body_point_jacobian = 
        tree.transformPointsJacobian<double>(robot_kinematics_cache, 
                    goal.body_point, goal.goal_frame, 0, false);

      if (goal.world_goal.empty()){
        printf("TODO: prune empty goal for index %d\n", goal.goal_frame);
        continue;
      }
      if (goal.world_goal.rows() != 3 && goal.world_goal.cols() != 1){
        printf("World goal has wrong dimension: %ld x %ld.\n", goal.world_goal.rows(), goal.world_goal.cols());
        continue;
      }

      Vector3d world_goal = goal.world_goal.value(t);

      Vector3d error = world_goal - body_point_in_world;
      VectorXd correction = body_point_jacobian.transpose() * error;
      q_robot += correction * dt * goal.weight;
    }
    //cout << "q robot " << q_robot.transpose() << endl;

    rm.updateRigidBodyTree(tree, q_robot, {"shadow_hand_sim_gt"});
    t += dt;
  }



  return 0;
}
