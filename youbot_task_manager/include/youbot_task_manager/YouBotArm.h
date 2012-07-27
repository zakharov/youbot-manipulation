/*
 * YouBotArm.h
 *
 *  Created on: Jun 19, 2012
 *      Author: clantos
 */

#ifndef YOUBOTARM_H_
#define YOUBOTARM_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <arm_navigation_msgs/MoveArmAction.h>


class YouBotArm {
public:
    YouBotArm();
    virtual ~YouBotArm();
    bool init(ros::NodeHandle nh);
    bool moveArm(geometry_msgs::PoseStamped goal_pose);
    bool moveArm(sensor_msgs::JointState goal_joints);
    void moveToJointPos(sensor_msgs::JointState joint_pos);
    void openGripper();
    void closeGripper();
    bool addGrippedObject();
    sensor_msgs::JointState getCurrentState() { return arm_joint_state; }

private:
    ros::Subscriber joint_sub;
    ros::Publisher armPositionsPublisher;
    ros::Publisher gripperPositionPublisher;
    ros::ServiceClient get_planning_scene_client;
    ros::ServiceClient set_planning_scene_client;
    ros::ServiceClient ik_client;
    ros::ServiceClient ik_solver_info_client;

    tf::TransformListener* tf_listener;

    actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> *move_arm;

    sensor_msgs::JointState arm_joint_state;
    sensor_msgs::JointState ik_solution;
    std::vector<geometry_msgs::PoseStamped> goal_targets;

    bool joint_pos_defined;

    bool init_joints();

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr & joint_state);

    bool requestInverseKinematics(geometry_msgs::PoseStamped pose);
    std::string errorCode(int code);

};

#endif /* YOUBOTARM_H_ */
