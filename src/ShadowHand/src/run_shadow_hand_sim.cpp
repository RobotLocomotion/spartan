#include "RemoteTreeViewerWrapper.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;

static inline double randrange(double min, double max){
  return (((double)rand()) / RAND_MAX)*(max - min) + min;
}

int main(int argc, char ** argv){
        // Visualize sim with the remote tree viewer
	RemoteTreeViewerWrapper rm;

	RigidBodyTree<double> tree;
	string shadow_hand_path;
	shadow_hand_path += getenv("SPARTAN_SOURCE_DIR");
	shadow_hand_path += "models/shadow_hand/hand/shadow_hand_lite.urdf";
	AddModelInstanceFromUrdfFileWithRpyJointToWorld(shadow_hand_path, &tree);
	Affine3d tf_robot;
	tf_robot.setIdentity();
	rm.publishRigidBodyTree(tree, VectorXd::Zero(tree.get_num_positions()), Vector4d(0.3, 0.3, 1.0, 1.0), {"shadow_hand_sim_gt"});

	return 0;
}
