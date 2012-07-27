#include <ros/ros.h>
#include "JointTrajectoryAction.h"
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>

JointTrajectoryAction* jointTrajecotryAction = NULL;

void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
    if (jointTrajecotryAction) {
        jointTrajecotryAction->execute(goal, as);
    }
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr joint_state) {
    if (jointTrajecotryAction) {
        jointTrajecotryAction->jointStateCallback(joint_state);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "youbot_joint_trajectory_action_server");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joint_states", 1, &jointStateCallback);

    ros::Publisher armVelocitiesPublisher;
    ros::Publisher armPositionsPublisher;

    armVelocitiesPublisher = n.advertise<brics_actuator::JointVelocities > ("arm_1/arm_controller/velocity_command", 1);
    armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);

    JointTrajectoryAction action(armVelocitiesPublisher, armPositionsPublisher, 1.0, 0.3, 100);
    jointTrajecotryAction = &action;

    Server server(n, "arm_1/arm_controller/joint_trajectory_action", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}
