/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 12/17/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 * Modified on 14/12/22
 *     Author: Tran Nguyen Le <tran.nguyenle@aalto.fi>
 */

#include <exercise3/functions.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "std_msgs/Float64.h"

#include <geometry_msgs/Transform.h>
#include <ros/ros.h>

using namespace std;


int main(int argc, char **argv) {
  ros::init(argc, argv, "pick_and_place");
  ros::NodeHandle node("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  double x, y, z;
  bool reset;
  node.getParam("x", x);
  node.getParam("y", y);
  node.getParam("z", z);
  node.getParam("reset", reset);

  // Wait for the simulator to come online and then we reset it
  if (reset) {
    std_srvs::Empty srv_reset;
    ros::service::waitForService("/lumi_mujoco/reset");
    ros::service::call("/lumi_mujoco/reset", srv_reset);
  }
  // Load MoveGroup interface and moveit visual tools
  moveit::planning_interface::MoveGroupInterface g_arm("lumi_arm");
  moveit::planning_interface::MoveGroupInterface g_hand("lumi_hand");
  moveit_visual_tools::MoveItVisualTools vis("base_link");
  vis.loadMarkerPub(true);
  vis.deleteAllMarkers();
  vis.trigger();

  // Get Start state
  const robot_state::RobotStatePtr state_ptr = g_arm.getCurrentState(10.0);
  if (!state_ptr) {
    ROS_ERROR_STREAM("Cannot get current state");
    return -1;
  }
  robot_state::RobotState state = *state_ptr;

  const moveit::core::JointModelGroup *jmg = state.getJointModelGroup(
      "lumi_arm"); // joint model group used for IK computation

  const std::string ee_link = "lumi_ee"; // Name of the end effector link
  const Eigen::Isometry3d arm_to_ee =
      state.getGlobalLinkTransform(g_arm.getEndEffectorLink()).inverse() *
      state.getGlobalLinkTransform(
          ee_link); // Transformation from base link to end effector link


  geometry_msgs::TransformStamped place_to_base_transform;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);

  place_to_base_transform.header.stamp = ros::Time::now();
  place_to_base_transform.header.frame_id = "base_link";
  place_to_base_transform.child_frame_id = "place0";
  place_to_base_transform.transform.translation.x = x;
  place_to_base_transform.transform.translation.y = y;
  place_to_base_transform.transform.translation.z = z;
  place_to_base_transform.transform.rotation.x = q.x();
  place_to_base_transform.transform.rotation.y = q.y();
  place_to_base_transform.transform.rotation.z = q.z();
  place_to_base_transform.transform.rotation.w = q.w();
  
  //Convert poses (transformation type) to poses (Eigen type) for mathematic manipulation
  Eigen::Isometry3d place_pose_eigen;
  tf::transformMsgToEigen(place_to_base_transform.transform, place_pose_eigen);


  place_pose_eigen = state.getGlobalLinkTransform("base_link") * place_pose_eigen * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  Eigen::Isometry3d pre_place_pose_eigen = place_pose_eigen * Eigen::Translation3d(0.0, 0.0, -0.1); 


  vis.publishAxis(place_pose_eigen);
  vis.publishAxis(pre_place_pose_eigen);

  vis.trigger();

  std::vector<moveit_msgs::RobotTrajectory> trajectories;





  // =============== PREGRASP to PREPLACE ===================
  g_arm.setStartState(state);
  if (!state.setFromIK(jmg, pre_place_pose_eigen, ee_link)) {
     ROS_ERROR_STREAM("Cannot set arm position with IK");
     return -1;
  }
  trajectories.push_back(planToState(g_arm, state));
  if (trajectories.back().joint_trajectory.points.empty()) {
     return -1;
  }
  state.setVariablePositions(
    trajectories.back().joint_trajectory.joint_names,
    trajectories.back().joint_trajectory.points.back().positions);
  // ==============================================


  // =============== PREPLACE to PLACE ===================
  geometry_msgs::Pose pre_place_pose;
  tf::poseEigenToMsg(pre_place_pose_eigen * arm_to_ee.inverse(), pre_place_pose);
  geometry_msgs::Pose place_pose;
  tf::poseEigenToMsg(place_pose_eigen * arm_to_ee.inverse(), place_pose);

  g_arm.setStartState(state);
  moveit_msgs::RobotTrajectory rtraj3;
  const double d3 = g_arm.computeCartesianPath({place_pose}, 0.01, 1.4, rtraj3);
  if (d3 < 0.99) {
     ROS_ERROR_STREAM("Cannot interpolate to the place position");
     return -1;
  }
  trajectories.push_back(rtraj3);

  state.setVariablePositions(
    trajectories.back().joint_trajectory.joint_names,
    trajectories.back().joint_trajectory.points.back().positions);
  // ==============================================


  // =============== PLACE to PREPLACE ===================
  g_arm.setStartState(state);
  moveit_msgs::RobotTrajectory rtraj4;
  const double d4 = g_arm.computeCartesianPath({pre_place_pose}, 0.01, 1.4, rtraj4);
  if (d4 < 0.99) {
     ROS_ERROR_STREAM("Cannot interpolate to the preplace position");
     return -1;
  }
  trajectories.push_back(rtraj4);

  state.setVariablePositions(
    trajectories.back().joint_trajectory.joint_names,
    trajectories.back().joint_trajectory.points.back().positions);
   // ==============================================


  // Visualise all trajectories
  for (const moveit_msgs::RobotTrajectory &t : trajectories) {
    vis.publishTrajectoryLine(t, state.getLinkModel(ee_link), jmg);
  }

  vis.trigger();
  g_hand.setStartStateToCurrentState();

  g_arm.execute(trajectoryToPlan(trajectories[0]));
  ros::Duration(1.0).sleep(); 
  g_arm.execute(trajectoryToPlan(trajectories[1]));
  ros::Duration(1.0).sleep(); 
  g_hand.execute(trajectoryToPlan(getGripperTrajectory(g_hand, true))); 
  ros::Duration(1.0).sleep();
  g_arm.execute(trajectoryToPlan(trajectories[2]));
  ros::Duration(1.0).sleep();
  g_hand.execute(trajectoryToPlan(getGripperTrajectory(g_hand, false)));
  ros::Duration(1.0).sleep();



  return 0;
}


