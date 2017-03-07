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
	RemoteTreeViewerWrapper rm;

	int n_points = 1000;
	double x_size = 3.0;
	double y_size = 2.0;
	double z_size = 1.0;
	Matrix3Xd pts(3, n_points);
	for (int i=0; i<n_points; i++){
		Vector3d xyz = Vector3d(
				randrange(-1.0, 1.0),
				randrange(-1.0, 1.0),
				randrange(-1.0, 1.0)
			);
		xyz /= xyz.norm();
		xyz[0] *= x_size;
		xyz[1] *= y_size;
		xyz[2] *= z_size;
		pts.col(i) = xyz;
	}
	rm.publishPointCloud(pts, {"test_pc"});
	rm.publishLine(pts, {"test_line"});


	Vector4d color_gray(0.7, 0.7, 0.7, 0.9);
	Vector4d color_blue(0.3, 0.3, 1.0, 0.9);

	Affine3d tf_box;
	tf_box.setIdentity();
	tf_box.translation()[1] = y_size*2;
	rm.publishGeometry(DrakeShapes::Box( {x_size, y_size, z_size} ), 
					   tf_box,
					   color_gray,
					   {"test_box"});

	Affine3d tf_sphere;
	tf_sphere.setIdentity();
	tf_sphere.translation()[1] = y_size*2 *2;
	rm.publishGeometry(DrakeShapes::Sphere( y_size ), 
					   tf_sphere,
					   color_gray,
					   {"test_sphere"});

	Affine3d tf_cylinder;
	tf_cylinder.setIdentity();
	tf_cylinder.translation()[1] = y_size*2 *3;
	rm.publishGeometry(DrakeShapes::Cylinder( y_size, z_size ), 
					   tf_cylinder,
					   color_gray,
					   {"test_cylinder"});

	Affine3d tf_capsule;
	tf_capsule.setIdentity();
	tf_capsule.translation()[1] = y_size*2 *4;
	rm.publishGeometry(DrakeShapes::Capsule( y_size, z_size ), 
					   tf_capsule,
					   color_gray,
					   {"test_capsule"});

	RigidBodyTree<double> tree;
	string iiwa_path;
	iiwa_path += getenv("SPARTAN_SOURCE_DIR");
	iiwa_path += "/drake/drake/examples/kuka_iiwa_arm/models/iiwa14/iiwa14.urdf";
	AddModelInstanceFromUrdfFileWithRpyJointToWorld(iiwa_path, &tree);
	Affine3d tf_robot;
	tf_robot.setIdentity();
	tf_robot.translation()[1] = -y_size*2;
	rm.publishRigidBodyTree(tree, VectorXd::Zero(tree.get_num_positions()), color_blue, {"test_robot"});

	return 0;
}