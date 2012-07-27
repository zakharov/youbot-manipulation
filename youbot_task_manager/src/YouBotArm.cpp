/*
 * YouBotArm.cpp
 *
 *  Created on: Jun 19, 2012
 *      Author: clantos
 */

#include <youbot_task_manager/YouBotArm.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <arm_navigation_msgs/utils.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <tf/transform_datatypes.h>
#include <brics_actuator/JointPositions.h>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>

/*****************************************************************************
** Implementation of Class
*****************************************************************************/

YouBotArm::YouBotArm() {
    joint_pos_defined = false;
}

YouBotArm::~YouBotArm() {
}

/*****************************************************************************
** Public Functions
*****************************************************************************/

bool YouBotArm::init(ros::NodeHandle nh)
{
    joint_sub = nh.subscribe("joint_states",1,&YouBotArm::jointStateCallback,this);
    armPositionsPublisher = nh.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
    gripperPositionPublisher = nh.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);
    get_planning_scene_client = nh.serviceClient<arm_navigation_msgs::GetPlanningScene>("environment_server/get_planning_scene");
    set_planning_scene_client = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>("environment_server/set_planning_scene_diff");
    ik_client = nh.serviceClient<kinematics_msgs::GetPositionIK>("youbot_arm_kinematics/get_ik");
    ik_solver_info_client = nh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("youbot_arm_kinematics/get_ik_solver_info");
    tf_listener = new tf::TransformListener(nh);
    move_arm = new actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction>("move_arm",true);
    if (init_joints())
    {
        ROS_INFO("Youbot Arm Ready");
        return true;
    }
    return false;
}

bool YouBotArm::moveArm(geometry_msgs::PoseStamped goal_pose)
{
    for (int i=0; i<5;i++)
    {
        if (requestInverseKinematics(goal_pose))
        {
        	moveArm(ik_solution);
        	return true;
        }
    }
    return false;
}

bool YouBotArm::moveArm(sensor_msgs::JointState goal_joints)
{
    arm_navigation_msgs::MoveArmGoal goal;
    goal.motion_plan_request.group_name = "arm";
    goal.motion_plan_request.num_planning_attempts = 1;
    goal.motion_plan_request.planner_id = std::string("");
    goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
    goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

    goal.motion_plan_request.goal_constraints.joint_constraints.resize(goal_joints.name.size());
    for (uint i = 0; i < goal_joints.name.size(); i++)
    {
    	goal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = goal_joints.name.at(i);
    	goal.motion_plan_request.goal_constraints.joint_constraints[i].position = goal_joints.position.at(i);
    	goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    	goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
    }

    move_arm->sendGoal(goal);
    move_arm->waitForResult(ros::Duration(10.0));
    if ((move_arm->getResult()->error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS) ||
        (move_arm->getResult()->error_code.val == 0))
    {
        ROS_INFO("Trajectory Successful");
        return true;
    }
    ROS_WARN("Trajectory Not Successful with Result Code: %s",errorCode(move_arm->getResult()->error_code.val).c_str());
    return false;
}

void YouBotArm::moveToJointPos(sensor_msgs::JointState joint_pos)
{
    brics_actuator::JointPositions command;
    std::vector <brics_actuator::JointValue> armJointPositions;
    armJointPositions.resize(joint_pos.name.size());

    for (uint j = 0; j < joint_pos.name.size(); j++)
    {
        armJointPositions[j].joint_uri = joint_pos.name.at(j);
        armJointPositions[j].value = joint_pos.position.at(j);
        armJointPositions[j].unit = boost::units::to_string(boost::units::si::radians);
    }

    command.positions = armJointPositions;
    armPositionsPublisher.publish(command);
    ROS_INFO("Joint Position Published");
}

void YouBotArm::openGripper()
{
    brics_actuator::JointPositions command;
    std::vector <brics_actuator::JointValue> gripperJointPositions;
    gripperJointPositions.resize(2);

    gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
    gripperJointPositions[0].value = 0.011;
    gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

    gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
    gripperJointPositions[1].value = 0.011;
    gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

    command.positions = gripperJointPositions;
    gripperPositionPublisher.publish(command);
    ROS_INFO("Gripper opened");
}

void YouBotArm::closeGripper()
{
    brics_actuator::JointPositions command;
    std::vector <brics_actuator::JointValue> gripperJointPositions;
    gripperJointPositions.resize(2);

    gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
    gripperJointPositions[0].value = 0.001;
    gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

    gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
    gripperJointPositions[1].value = 0.001;
    gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

    command.positions = gripperJointPositions;
    gripperPositionPublisher.publish(command);
    ROS_INFO("Gripper Closed");
}

bool YouBotArm::addGrippedObject()
{
    arm_navigation_msgs::AttachedCollisionObject att_object;
    att_object.link_name = "gripper_finger_link_l";

    att_object.object.id = "gripped_objects";
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

    att_object.object.header.frame_id = "gripper_finger_link_l";
    att_object.object.header.stamp = ros::Time::now();
    arm_navigation_msgs::Shape object;
    object.type = arm_navigation_msgs::Shape::BOX;
    object.dimensions.resize(3);
    object.dimensions[0] = 0.02;
    object.dimensions[1] = 0.02;
    object.dimensions[2] = 0.02;
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    att_object.object.shapes.push_back(object);
    att_object.object.poses.push_back(pose);
    att_object.touch_links.push_back("gripper_finger_link_l");
    arm_navigation_msgs::GetPlanningScene planning_scene;
//    planning_scene.request.planning_scene_diff.attached_collision_objects.push_back(att_object);
//    if (get_planning_scene_client.waitForExistence(ros::Duration(1.0)))
//    {
//    	if(!get_planning_scene_client.call(planning_scene))
//    	{
//    		ROS_WARN("Can't get planning scene");
//    		return false;
//    	}
//    }
//    else
//    {
//    	ROS_WARN("Get Planning scene does not exist");
//    }


    arm_navigation_msgs::SetPlanningSceneDiff srv;
    srv.request.planning_scene_diff.attached_collision_objects.push_back(att_object);
    if(!set_planning_scene_client.call(srv))
    {
    	ROS_WARN("Can't set planning scene");
    	return false;
    }

    ROS_INFO("Object Attached");
    ROS_INFO("OBJECTS FOUND: %d",srv.response.planning_scene.attached_collision_objects.size());
    return true;
}

/*****************************************************************************
** ROS Callback Functions
*****************************************************************************/

void YouBotArm::jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state)
{
    int k = arm_joint_state.name.size();
    for (uint i = 0; i < joint_state->name.size();i++)
    {
        for (uint j=0; j < arm_joint_state.name.size(); j++)
        {
            if (arm_joint_state.name.at(j) == joint_state->name.at(i))
            {
                arm_joint_state.position[j] = joint_state->position.at(i);
                k--;
                break;
            }
        }
        if (k == 0)
        {
            joint_pos_defined = true;
            break;
        }
    }
}

/*****************************************************************************
** Private Functions
*****************************************************************************/

bool YouBotArm::init_joints()
{
    while (!ik_solver_info_client.waitForExistence(ros::Duration(1.0)) && ros::ok());
    kinematics_msgs::GetKinematicSolverInfo srv;
    if (!ik_solver_info_client.call(srv))
    {
        ROS_ERROR("IK Solver Info call failed");
        return false;
    }
    arm_joint_state.name = srv.response.kinematic_solver_info.joint_names;
    arm_joint_state.position.resize(arm_joint_state.name.size());
    arm_joint_state.velocity.resize(arm_joint_state.name.size());
    arm_joint_state.effort.resize(arm_joint_state.name.size());
    return true;
}

bool YouBotArm::requestInverseKinematics(geometry_msgs::PoseStamped pose)
{
    kinematics_msgs::GetPositionIK::Request req;
    kinematics_msgs::GetPositionIK::Response resp;

    req.timeout = ros::Duration(5.0);
    req.ik_request.ik_link_name = "arm_link_5";

    if (pose.header.frame_id == "arm_link_0")
    {
        req.ik_request.pose_stamped = pose;
    }
    else
    {
        geometry_msgs::PoseStamped outPoint;
        if (!tf_listener->canTransform(pose.header.frame_id,"arm_link_0",ros::Time(0)))
        {
            ROS_ERROR("Transform between Pose given for IK and arm_link_0 doesn't exist");
            return false;
        }
        tf_listener->transformPose("arm_link_0",pose,outPoint);
        req.ik_request.pose_stamped = outPoint;
    }

    req.ik_request.ik_seed_state.joint_state = arm_joint_state;

    if (ik_client.call(req,resp))
    {
        ROS_INFO("Request Inverse Kinematics of Target Finished with Result: %s",errorCode(resp.error_code.val).c_str());

        ik_solution.name.resize(0);
        ik_solution.position.resize(0);
        ik_solution.velocity.resize(0);
        ik_solution.effort.resize(0);

        if (resp.error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS)
        {
            ik_solution = resp.solution.joint_state;
            return true;
        }
        return false;
    }
    ROS_ERROR("Inverse Kinematics Service Call failed");
    return false;
}

std::string YouBotArm::errorCode(int code)
{
    switch (code) {
    case arm_navigation_msgs::ArmNavigationErrorCodes::PLANNING_FAILED:
        return "Planning Failed";
    case arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS:
        return "Success";
    case arm_navigation_msgs::ArmNavigationErrorCodes::TIMED_OUT:
        return "Timeout";
    case arm_navigation_msgs::ArmNavigationErrorCodes::START_STATE_IN_COLLISION:
        return "Start State in collision";
    case arm_navigation_msgs::ArmNavigationErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "Start State violates path constraints";
    case arm_navigation_msgs::ArmNavigationErrorCodes::GOAL_IN_COLLISION:
        return "Goal in collision";
    case arm_navigation_msgs::ArmNavigationErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "Goal violates path constraints";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_ROBOT_STATE:
        return "Invalid Robot State";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INCOMPLETE_ROBOT_STATE:
        return "Incomplete Robot State";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_PLANNER_ID:
        return "Invalid Planner ID";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_NUM_PLANNING_ATTEMPTS:
        return "Invalid number of planning attemps";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_ALLOWED_PLANNING_TIME:
        return "Invalid allowed planning time";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_GROUP_NAME:
        return "Invalid group name";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_GOAL_JOINT_CONSTRAINTS:
        return "Invalid goal joint constraints";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_GOAL_POSITION_CONSTRAINTS:
        return "Invalid goal position constraints";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_GOAL_ORIENTATION_CONSTRAINTS:
        return "Invalid goal orientation constrains";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_PATH_JOINT_CONSTRAINTS:
        return "Invalid path joint constraints";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_PATH_POSITION_CONSTRAINTS:
        return "Invalid path position constraints";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_PATH_ORIENTATION_CONSTRAINTS:
        return "Invalid path orientation constraints";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_TRAJECTORY:
        return "Invalid trajectory";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_INDEX:
        return "Invalid index";
    case arm_navigation_msgs::ArmNavigationErrorCodes::JOINT_LIMITS_VIOLATED:
        return "Joint limits violated";
    case arm_navigation_msgs::ArmNavigationErrorCodes::PATH_CONSTRAINTS_VIOLATED:
        return "Path constraints violated";
    case arm_navigation_msgs::ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED:
        return "Collision constraints violated";
    case arm_navigation_msgs::ArmNavigationErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
        return "Goal constraints violated";
    case arm_navigation_msgs::ArmNavigationErrorCodes::JOINTS_NOT_MOVING:
        return "Joints not moving";
    case arm_navigation_msgs::ArmNavigationErrorCodes::TRAJECTORY_CONTROLLER_FAILED:
        return "Trajectory controller failed";
    case arm_navigation_msgs::ArmNavigationErrorCodes::FRAME_TRANSFORM_FAILURE:
        return "Frame transform failure";
    case arm_navigation_msgs::ArmNavigationErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
        return "Collision checking unavailable";
    case arm_navigation_msgs::ArmNavigationErrorCodes::ROBOT_STATE_STALE:
        return "Robot State stale";
    case arm_navigation_msgs::ArmNavigationErrorCodes::SENSOR_INFO_STALE:
        return "Sensor info stale";
    case arm_navigation_msgs::ArmNavigationErrorCodes::NO_IK_SOLUTION:
        return "No Inverse Kinematic solution";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_LINK_NAME:
        return "Invalid Link Name";
    case arm_navigation_msgs::ArmNavigationErrorCodes::IK_LINK_IN_COLLISION:
        return "Inverse Kinematic Link in collision";
    case arm_navigation_msgs::ArmNavigationErrorCodes::NO_FK_SOLUTION:
        return "No Forward Kinematic solution";
    case arm_navigation_msgs::ArmNavigationErrorCodes::KINEMATICS_STATE_IN_COLLISION:
        return "Kinematics State in collision";
    case arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_TIMEOUT:
        return "Invalid Timeout";
    default:
    	std::stringstream temp;
    	temp << "Uknown Result Code (" << code << ")";
        return temp.str();
    }
}
