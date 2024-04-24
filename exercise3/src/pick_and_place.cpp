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

#include <geometry_msgs/Transform.h>
#include <ros/ros.h>

int pick_and_place(int argc, char **argv, std::vector<double> pick_coords, std::vector<double> place_coords, bool reset) {
  ros::init(argc, argv, "pick_and_place");
  ros::NodeHandle node("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();

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

  geometry_msgs::TransformStamped pick_to_base_transform;
  geometry_msgs::TransformStamped place_to_base_transform;

  pick_to_base_transform.header.stamp = ros::Time::now();
  pick_to_base_transform.header.frame_id = "base_link";
  pick_to_base_transform.child_frame_id = "pick0";
  pick_to_base_transform.transform.translation.x = pick_coords[0];
  pick_to_base_transform.transform.translation.y = pick_coords[1];
  pick_to_base_transform.transform.translation.z = pick_coords[2];
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  pick_to_base_transform.transform.rotation.x = q.x();
  pick_to_base_transform.transform.rotation.y = q.y();
  pick_to_base_transform.transform.rotation.z = q.z();
  pick_to_base_transform.transform.rotation.w = q.w();

  place_to_base_transform.header.stamp = ros::Time::now();
  place_to_base_transform.header.frame_id = "base_link";
  place_to_base_transform.child_frame_id = "place0";
  place_to_base_transform.transform.translation.x = place_coords[0];
  place_to_base_transform.transform.translation.y = place_coords[1];
  place_to_base_transform.transform.translation.z = place_coords[2];
  place_to_base_transform.transform.rotation.x = q.x();
  place_to_base_transform.transform.rotation.y = q.y();
  place_to_base_transform.transform.rotation.z = q.z();
  place_to_base_transform.transform.rotation.w = q.w();
  
  //Convert poses (transformation type) to poses (Eigen type) for mathematic manipulation
  Eigen::Isometry3d grasp_pose_eigen;
  Eigen::Isometry3d place_pose_eigen;
  tf::transformMsgToEigen(pick_to_base_transform.transform, grasp_pose_eigen);
  tf::transformMsgToEigen(place_to_base_transform.transform, place_pose_eigen);

  




  /*TODO: Compute required poses and visualize them 
  (Hints: we show two examples here. (Use this as reference)
    - e1 represent for translation 0.1m in X axis while
      Eigen::Isometry3d e1 = state.getGlobalLinkTransform("base_link") * Eigen::Translation3d(0.1, 0.0, 0.0);
	  
	- e2 represents for translation -0.1m in X axis and rotation PI/2 around X axis)
      Eigen::Isometry3d e2 = state.getGlobalLinkTransform("base_link")  * Eigen::Translation3d(-0.1, 0.0, 0.0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
	  
	  We already have place_pose written for you. Please use it as reference to fill in grasp_pose and pre_grasp_pose
      // Hints: you can use this function to visualize the frames/ pose. 
      vis.publishAxis(e1);
      vis.publishAxis(e2);
  */ 
  // =============== Your code ===================  
     grasp_pose_eigen = state.getGlobalLinkTransform("base_link") * grasp_pose_eigen * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
     place_pose_eigen = state.getGlobalLinkTransform("base_link") * place_pose_eigen * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
     Eigen::Isometry3d pre_grasp_pose_eigen = grasp_pose_eigen * Eigen::Translation3d(0.0, 0.0, -0.1); 
	 Eigen::Isometry3d pre_place_pose_eigen = place_pose_eigen * Eigen::Translation3d(0.0, 0.0, -0.1); 


     vis.publishAxis(grasp_pose_eigen);
     vis.publishAxis(place_pose_eigen);
     vis.publishAxis(pre_grasp_pose_eigen);
	 vis.publishAxis(pre_place_pose_eigen);

  // ==============================================
  vis.trigger();

  std::vector<moveit_msgs::RobotTrajectory> trajectories;


  // =============== START to PREGRASP =================== 
  g_arm.setStartState(state);
  if (!state.setFromIK(jmg, pre_grasp_pose_eigen, ee_link)) {
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


  // =============== PREGRASP to GRASP =================== 
  g_arm.setStartState(state);
  geometry_msgs::Pose pre_grasp_pose;
  tf::poseEigenToMsg(pre_grasp_pose_eigen * arm_to_ee.inverse(), pre_grasp_pose);
  geometry_msgs::Pose grasp_pose;
  tf::poseEigenToMsg(grasp_pose_eigen * arm_to_ee.inverse(), grasp_pose);

  moveit_msgs::RobotTrajectory rtraj;
  const double d = g_arm.computeCartesianPath({grasp_pose}, 0.01, 1.4, rtraj);
  if (d < 0.99) {
     ROS_ERROR_STREAM("Cannot interpolate to the grasping position");
     return -1;
  }
  trajectories.push_back(rtraj);

  state.setVariablePositions(
      trajectories.back().joint_trajectory.joint_names,
      trajectories.back().joint_trajectory.points.back().positions);
  // ==============================================


  // =============== GRASP to PREGRASP =================== 
  g_arm.setStartState(state);
  moveit_msgs::RobotTrajectory rtraj2;
  const double d2 = g_arm.computeCartesianPath({pre_grasp_pose}, 0.01, 1.4, rtraj2);
  if (d2 < 0.99) {
     ROS_ERROR_STREAM("Cannot interpolate to the pregrasping position");
     return -1;
  }
  trajectories.push_back(rtraj2);

  state.setVariablePositions(
    trajectories.back().joint_trajectory.joint_names,
    trajectories.back().joint_trajectory.points.back().positions);
  // ==============================================



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


  // =============== PREPLACE to PLACE ===================
  geometry_msgs::Pose pre_place_pose;
  tf::poseEigenToMsg(pre_place_pose_eigen * arm_to_ee.inverse(), pre_place_pose);
  geometry_msgs::Pose place_pose;
  tf::poseEigenToMsg(place_pose_eigen * arm_to_ee.inverse(), place_pose);

  g_arm.setStartState(state);
  moveit_msgs::RobotTrajectory rtraj3;
  const double d3 = g_arm.computeCartesianPath({place_pose}, 0.01, 1.4, rtraj3);
  if (d3 < 0.99) {
     ROS_ERROR_STREAM("Cannot interpolate to the pregrasping position");
     return -1;
  }
  trajectories.push_back(rtraj3);

  state.setVariablePositions(
    trajectories.back().joint_trajectory.joint_names,
    trajectories.back().joint_trajectory.points.back().positions);


  // =============== PLACE to PREPLACE ===================
  g_arm.setStartState(state);
  moveit_msgs::RobotTrajectory rtraj4;
  const double d4 = g_arm.computeCartesianPath({pre_place_pose}, 0.01, 1.4, rtraj4);
  if (d4 < 0.99) {
     ROS_ERROR_STREAM("Cannot interpolate to the pregrasping position");
     return -1;
  }
  trajectories.push_back(rtraj4);

  state.setVariablePositions(
    trajectories.back().joint_trajectory.joint_names,
    trajectories.back().joint_trajectory.points.back().positions);
 

  // Visualise all trajectories
  for (const moveit_msgs::RobotTrajectory &t : trajectories) {
    vis.publishTrajectoryLine(t, state.getLinkModel(ee_link), jmg);
  }
  vis.trigger();

  if (true) {
    g_hand.setStartStateToCurrentState();

    g_arm.execute(trajectoryToPlan(trajectories[0])); // Execute trajectory to PRE_GRASP
    ros::Duration(1.0).sleep(); 
    g_hand.execute(trajectoryToPlan(getGripperTrajectory(g_hand, true))); // OPEN GRIPPER
    
    ros::Duration(1.0).sleep();
    g_arm.execute(trajectoryToPlan(trajectories[1]));
    ros::Duration(1.0).sleep();
    g_hand.execute(trajectoryToPlan(getGripperTrajectory(g_hand, false)));
    ros::Duration(1.0).sleep();
    g_arm.execute(trajectoryToPlan(trajectories[2]));
    ros::Duration(1.0).sleep();
    g_arm.execute(trajectoryToPlan(trajectories[3]));
    ros::Duration(1.0).sleep();
	g_arm.execute(trajectoryToPlan(trajectories[4]));
	ros::Duration(1.0).sleep();
    g_hand.execute(trajectoryToPlan(getGripperTrajectory(g_hand, true)));
    ros::Duration(1.0).sleep();
	g_arm.execute(trajectoryToPlan(trajectories[5]));
    ros::Duration(1.0).sleep();

  } else {
     return -1;
  }


  return 0;
}
int main(int argc, char **argv) {
	//std::vector<double> pick_coords({0.55, 0.0, 0.025});
	//std::vector<double> place_coords({0.0, -0.5, 0.025});
	//pick_and_place(argc, argv, pick_coords, place_coords, true);
	double pick_x;
	double pick_y;
	double pick_z;
	double place_x;
	double place_y;
	double place_z;
	bool reset = true;
    bool continue_;
	std::cout << "Continue?" << std::endl;
	std::cin >> continue_;
	while (continue_) {
		std::cout << "Enter pick coordinate x?" << std::endl;
		std::cin >> pick_x;
		std::cout << "Enter pick coordinate y?" << std::endl;
		std::cin >> pick_y;
		std::cout << "Enter pick coordinate z?" << std::endl;
		std::cin >> pick_z;
		std::cout << "Enter place coordinate x?" << std::endl;
		std::cin >> place_x;
		std::cout << "Enter place coordinate y?" << std::endl;
		std::cin >> place_y;
		std::cout << "Enter place coordinate z?" << std::endl;
		std::cin >> place_z;

		std::vector<double> pick_coords({pick_x, pick_y, pick_z});
	    std::vector<double> place_coords({place_x, place_y, place_z});
		pick_and_place(argc, argv, pick_coords, place_coords, reset);

		std::cout << "Continue?" << std::endl;
		std::cin >> continue_;
		reset = false;
	}

}
