#include "drake_iiwa_sim/ros_scene_graph_visualizer.h"

#include "drake/multibody/shapes/geometry.h"
#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/quaternion.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake_lcmtypes/drake/lcmt_viewer_load_robot.hpp"

namespace drake_iiwa_sim {

using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using drake::systems::Context;
using drake::systems::EventStatus;
using drake::systems::rendering::PoseBundle;
using drake::systems::Value;
using Eigen::Quaternion;
using DrakeShapes::Mesh;
using DrakeShapes::TrianglesVector;
using DrakeShapes::PointsVector;

/// Heavily references Drake's meshcat_visualizer,
/// especially for the initialization / load hack
/// that abuses the LCM draw_robot call to do the
/// scenegraph-interaction heavy lifting.
RosSceneGraphVisualizer::RosSceneGraphVisualizer(
                          const SceneGraph<double>& scene_graph,
                          std::string server_name,
                          double draw_period)
    : server_(server_name),
      scene_graph_(scene_graph),
      pose_bundle_input_port_(DeclareAbstractInputPort(
          drake::systems::kUseDefaultName, Value<PoseBundle<double>>()).get_index()),
      cheat_pose_publisher_(nh_.advertise<geometry_msgs::PoseStamped>(
          "/dynamic_gt_object_pose", 1))
      {
  DeclareInitializationPublishEvent(&RosSceneGraphVisualizer::DoInitialization);
  DeclarePeriodicPublishEvent(draw_period, 0.0, &RosSceneGraphVisualizer::DoPeriodicPublish);
}

std::string RosSceneGraphVisualizer::MakeFullName(const std::string& input_name,
                                                  int robot_num) const {
  std::string source_name, frame_name;
  if (input_name == "world") {
    // Not sure the history of this, but all geometry *but*
    // world geometry comes in with "source_name::frame_name",
    // while world geometry is just named "world".
    source_name = "world";
    frame_name = "world";
  } else {
    auto offset = input_name.find("::");
    if (offset == std::string::npos) {
      return "";
    }
    source_name = input_name.substr(0, offset);
    frame_name = input_name.substr(offset + 2);
  }

  std::stringstream full_name;
  full_name << source_name << "::" << robot_num << "::" << frame_name;
  return full_name.str();
}

EventStatus RosSceneGraphVisualizer::DoInitialization(const Context<double>& context) const {
  drake::lcm::DrakeMockLcm mock_lcm;
  drake::geometry::DispatchLoadMessage(scene_graph_, &mock_lcm);
  auto load_robot_msg = mock_lcm.DecodeLastPublishedMessageAs
    <drake::lcmt_viewer_load_robot>("DRAKE_VIEWER_LOAD_ROBOT");

  for (const auto link : load_robot_msg.link){
    std::string full_name = MakeFullName(link.name, link.robot_num);
    if (full_name == ""){
      printf("Couldn't find separator, this name is malformed. Skipping...\n");
      printf("Name in question was: %s\n", link.name.c_str());
      continue;
    }

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base";
    int_marker.header.stamp = ros::Time();
    int_marker.name = full_name.c_str();

    // Hack city: Pete's doing to only publish the sugar
    std::cout << int_marker.name << std::endl;
    if (int_marker.name.find("sugar") != std::string::npos) {
       std::cout << "found a sugar!" << std::endl;
       std::cout << "it was: " << int_marker.name << std::endl;
    } else {
      std::cout << "not adding" << int_marker.name << std::endl;
      continue;
    }
    // Hack city: Pete's doing to only publish the sugar

    visualization_msgs::InteractiveMarkerControl control_marker;
    control_marker.always_visible = true;

    for (const auto geom : link.geom){
      // MBT current sets alpha = 0 to make collision geometry
      // "invisible", so don't draw them.
      if (geom.color[3] == 0){
        continue;
      }

      visualization_msgs::Marker geom_marker;
      switch (geom.type){
        case geom.BOX:
          if (geom.num_float_data != 3){
            printf("Malformed geom.BOX, skipping.");
            break;
          }
          geom_marker.type = visualization_msgs::Marker::CUBE;
          geom_marker.scale.x = geom.float_data[0];
          geom_marker.scale.y = geom.float_data[1];
          geom_marker.scale.z = geom.float_data[2];
          geom_marker.color.r = geom.color[0];
          geom_marker.color.g = geom.color[1];
          geom_marker.color.b = geom.color[2];
          geom_marker.color.a = geom.color[3];
          break;
        case geom.SPHERE:
          if (geom.num_float_data != 1){
            printf("Malformed geom.SPHERE, skipping.");
            break;
          }
          geom_marker.type = visualization_msgs::Marker::SPHERE;
          geom_marker.scale.x = geom.float_data[0];
          geom_marker.scale.y = geom.float_data[0];
          geom_marker.scale.z = geom.float_data[0];
          geom_marker.color.r = geom.color[0];
          geom_marker.color.g = geom.color[1];
          geom_marker.color.b = geom.color[2];
          geom_marker.color.a = geom.color[3];
          break;
        case geom.CYLINDER:
          if (geom.num_float_data != 2){
            printf("Malformed geom.CYLINDER, skipping.");
            break;
          }
          geom_marker.type = visualization_msgs::Marker::CYLINDER;
          geom_marker.scale.x = geom.float_data[1];
          geom_marker.scale.y = geom.float_data[1];
          geom_marker.scale.z = geom.float_data[0];
          geom_marker.color.r = geom.color[0];
          geom_marker.color.g = geom.color[1];
          geom_marker.color.b = geom.color[2];
          geom_marker.color.a = geom.color[3];
          break;
        case geom.MESH:
          if (geom.num_float_data != 3 || geom.string_data.size() < 4){
            printf("Malformed geom.MESH, skipping.");
            break;
          }

          // Unfortunately Rviz seems to only want package-relative
          // paths, not absolute paths. So instead, load in the appropriate
          // geometry and send it over as triangles.
          {
            Mesh mesh("", geom.string_data.substr(
                      0, geom.string_data.size()-3) + std::string("obj"));
            PointsVector points;
            TrianglesVector faces;
            mesh.LoadObjFile(&points, &faces);

            geom_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            geom_marker.scale.x = geom.float_data[0];
            geom_marker.scale.y = geom.float_data[1];
            geom_marker.scale.z = geom.float_data[2];
            geom_marker.color.r = geom.color[0];
            geom_marker.color.g = geom.color[1];
            geom_marker.color.b = geom.color[2];
            geom_marker.color.a = geom.color[3];
            for (const auto face : faces){
              for (int i=0; i<3; i++) {
                const Eigen::Vector3d& pt = points[face[i]];
                geom_marker.points.emplace_back();
                geometry_msgs::Point * new_point_msg = &geom_marker.points.back();
                new_point_msg->x = pt[0];
                new_point_msg->y = pt[1];
                new_point_msg->z = pt[2]; 
              }
            }
          }
          break;
        default:
          printf("UNSUPPORTED GEOMETRY TYPE %d IGNORED\n", geom.type);
          break;
      }
      // Set geom marker offset in its parent marker
      geom_marker.pose.position.x = geom.position[0];
      geom_marker.pose.position.y = geom.position[1];
      geom_marker.pose.position.z = geom.position[2];
      geom_marker.pose.orientation.w = geom.quaternion[0];
      geom_marker.pose.orientation.x = geom.quaternion[1];
      geom_marker.pose.orientation.y = geom.quaternion[2];
      geom_marker.pose.orientation.z = geom.quaternion[3];
      control_marker.markers.push_back(geom_marker);
    }
    int_marker.controls.push_back(control_marker); 
    server_.insert(int_marker);
  }
  server_.applyChanges();
  return EventStatus::Succeeded();
}

EventStatus RosSceneGraphVisualizer::DoPeriodicPublish(
    const Context<double>& context) const {
  
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "base";

  const drake::systems::AbstractValue* input =
      this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& pose_bundle = input->GetValue<PoseBundle<double>>();

  for (int frame_i = 0; frame_i < pose_bundle.get_num_poses(); frame_i++) {
    const RigidTransform<double> tf(pose_bundle.get_pose(frame_i));
    const Eigen::Vector3d t = tf.translation();
    const Quaternion<double> q = tf.rotation().ToQuaternion();
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = t[0];
    pose_msg.position.y = t[1];
    pose_msg.position.z = t[2];
    pose_msg.orientation.w = q.w();
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();


    int robot_num = pose_bundle.get_model_instance_id(frame_i);
    std::string full_name = MakeFullName(pose_bundle.get_name(frame_i), robot_num);
    server_.setPose(full_name, pose_msg, header);

    if (full_name.find("sugar") != std::string::npos) {
      geometry_msgs::PoseStamped pose_stamped_msg;
      pose_stamped_msg.header = header;
      pose_stamped_msg.pose = pose_msg;
      cheat_pose_publisher_.publish(pose_stamped_msg);
    } 

  }
  server_.applyChanges();
  ros::spinOnce();
  return EventStatus::Succeeded();
}

}  // namespace drake_iiwa_sim