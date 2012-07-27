/*
 * task_manager.cpp
 *
 *  Created on: Jun 19, 2012
 *      Author: clantos
 */

#include <ros/ros.h>
#include <youbot_task_manager/YouBotArm.h>
#include <planning_environment/models/collision_models.h>
#include <planning_models/kinematic_state.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/thread.hpp>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

void spinThread()
{
  ros::spin();
}

void visualizeRobot(sensor_msgs::JointState current_state,
						 ros::Publisher vis_marker_array_publisher)
{
	planning_environment::CollisionModels cm("robot_description");
		std::vector<std::string> lnames;
		lnames.push_back("arm_link_0");
		lnames.push_back("arm_link_1");
		lnames.push_back("arm_link_2");
		lnames.push_back("arm_link_3");
		lnames.push_back("arm_link_4");
		lnames.push_back("arm_link_5");
		lnames.push_back("gripper_palm_link");
		lnames.push_back("gripper_finger_link_l");
		lnames.push_back("gripper_finger_link_r");
		std_msgs::ColorRGBA bad_color;
		bad_color.a = 0.6;
		bad_color.r = 1.0;
		bad_color.g = 0.0;
		bad_color.b = 0.0;
		std::map<std::string, double> state_map;
		for (uint i = 0; i < current_state.name.size(); i++)
		{
			state_map.insert(std::make_pair(current_state.name.at(i),
					current_state.position.at(i)));
		}

		planning_models::KinematicState state(cm.getKinematicModel());
		state.setKinematicState(state_map);
		visualization_msgs::MarkerArray arr;

		cm.getRobotMarkersGivenState(state, arr, bad_color,
				"arm", ros::Duration(600.0), &lnames,1.0, false);
		bad_color.r = 0.0;
		bad_color.g = 1.0;
		bad_color.b = 0.0;

		cm.getAttachedCollisionObjectMarkers(state,
				arr,
				"gripped_objects",
				bad_color,
				ros::Duration(600.0),
				false,
				&lnames);
		std::vector<std::string> object_names;
		cm.getAttachedCollisionObjectNames(object_names);
		if (object_names.size() != 0)
		{
			std::cout << object_names.front() << std::endl;
		}
		vis_marker_array_publisher.publish(arr);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "youbot_task_manager");
	boost::thread spin_thread(&spinThread);
	ros::NodeHandle n;

	YouBotArm youbot_arm;
	if (!youbot_arm.init(n))
	{
		return -1;
	}

	ros::Publisher vis_marker_publisher = n.advertise<visualization_msgs::Marker> ("youbot_trajectory_marker", 128);
	ros::Publisher vis_marker_array_publisher = n.advertise<visualization_msgs::MarkerArray> ("youbot_trajectory_marker_array", 128);
	ros::ServiceClient set_planning_scene_client = n.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>("environment_server/set_planning_scene_diff");

	ros::Duration(1.0).sleep();

	sensor_msgs::JointState starting_pos;
	starting_pos.name.push_back("arm_joint_1");
	starting_pos.position.push_back(2.56244);
	starting_pos.name.push_back("arm_joint_2");
	starting_pos.position.push_back(1.04883);
	starting_pos.name.push_back("arm_joint_3");
	starting_pos.position.push_back(-2.43523);
	starting_pos.name.push_back("arm_joint_4");
	starting_pos.position.push_back(1.73184);
	starting_pos.name.push_back("arm_joint_5");
	starting_pos.position.push_back(0.2);

	youbot_arm.moveToJointPos(starting_pos);

	ros::Duration(5.0).sleep();

	youbot_arm.openGripper();

	ros::Duration(1.0).sleep();

//	geometry_msgs::PoseStamped pick_pos;
//
//	pick_pos.header.frame_id = "arm_link_0";
//	pick_pos.pose.position.x = 0.094;
//	pick_pos.pose.position.y = -0.393;
//	pick_pos.pose.position.z = 0.000;
//	pick_pos.pose.orientation.x = 0.627;
//	pick_pos.pose.orientation.y = 0.666;
//	pick_pos.pose.orientation.z = -0.242;
//	pick_pos.pose.orientation.w = 0.325;

	sensor_msgs::JointState pick_pos;
	pick_pos.name.push_back("arm_joint_1");
	pick_pos.position.push_back(4.73709881491004);
	pick_pos.name.push_back("arm_joint_2");
	pick_pos.position.push_back(2.2569648588513647);
	pick_pos.name.push_back("arm_joint_3");
	pick_pos.position.push_back(-1.5802952248884407);
	pick_pos.name.push_back("arm_joint_4");
	pick_pos.position.push_back(2.613583848867441);
	pick_pos.name.push_back("arm_joint_5");
	pick_pos.position.push_back(3.017588239340703);

	if (!youbot_arm.moveArm(pick_pos))
	{
		ros::shutdown();
		spin_thread.join();
		return -1;
	}

	ros::Duration(1.0).sleep();

	youbot_arm.closeGripper();
	youbot_arm.addGrippedObject();
	visualizeRobot(youbot_arm.getCurrentState(),vis_marker_array_publisher);
	arm_navigation_msgs::SetPlanningSceneDiff srv;
	if(!set_planning_scene_client.call(srv))
	{
		ROS_WARN("Can't set planning scene");
		return false;
	}
	ROS_INFO("OBJECTS FOUND: %d",srv.response.planning_scene.attached_collision_objects.size());

	ros::Duration(1.0).sleep();

//	geometry_msgs::PoseStamped middle_pos;
//
//	middle_pos.header.frame_id = "arm_link_0";
//	middle_pos.pose.position.x = 0.040;
//	middle_pos.pose.position.y = 0.007;
//	middle_pos.pose.position.z = 0.434;
//	middle_pos.pose.orientation.x = -0.014;
//	middle_pos.pose.orientation.y = -0.006;
//	middle_pos.pose.orientation.z = 1.000;
//	middle_pos.pose.orientation.w = 0.007;
//
//	if (!youbot_arm.movearm(middle_pos))
//		{
//			return -1;
//		}

	geometry_msgs::PoseStamped place_pos;

	place_pos.header.frame_id = "arm_link_0";
	place_pos.pose.position.x = -0.206;
	place_pos.pose.position.y = -0.064;
	place_pos.pose.position.z = 0.132;
	place_pos.pose.orientation.x = -0.001;
	place_pos.pose.orientation.y = 0.977;
	place_pos.pose.orientation.z = -0.057;
	place_pos.pose.orientation.w = -0.205;

	if (!youbot_arm.moveArm(place_pos))
	{
		ros::shutdown();
		spin_thread.join();
		return -1;
	}

	ros::Duration(1.0).sleep();

	youbot_arm.openGripper();

	ros::Duration(1.0).sleep();

	geometry_msgs::PoseStamped finish_pos;

	finish_pos.header.frame_id = "arm_link_0";
	finish_pos.pose.position.x = 0.040;
	finish_pos.pose.position.y = 0.007;
	finish_pos.pose.position.z = 0.534;
	finish_pos.pose.orientation.x = -0.014;
	finish_pos.pose.orientation.y = -0.006;
	finish_pos.pose.orientation.z = 1.000;
	finish_pos.pose.orientation.w = 0.007;

	if (!youbot_arm.moveArm(finish_pos))
	{
		ros::shutdown();
		spin_thread.join();
		return -1;
	}

	visualizeRobot(youbot_arm.getCurrentState(),vis_marker_array_publisher);
	if(!set_planning_scene_client.call(srv))
	{
		ROS_WARN("Can't set planning scene");
		return false;
	}
	ROS_INFO("OBJECTS FOUND: %d",srv.response.planning_scene.attached_collision_objects.size());

	ros::shutdown();
	spin_thread.join();
	return 0;
}
