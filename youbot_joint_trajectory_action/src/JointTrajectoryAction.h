#ifndef JOINTTRAJECTORYACTION_H
#define	JOINTTRAJECTORYACTION_H

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>

namespace KDL {
class Trajectory_Composite;
}
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

class JointTrajectoryAction {
public:
    JointTrajectoryAction(const ros::Publisher& armVelocitiesPublisher, const ros::Publisher& armPositionsPublisher);
    JointTrajectoryAction(const ros::Publisher& armVelocitiesPublisher, const ros::Publisher& armPositionsPublisher, 
        double velocityGain, double positionGain, double frequency);
    JointTrajectoryAction(const JointTrajectoryAction& orig);
    virtual ~JointTrajectoryAction();
    
    void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as);

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr joint_state);
    
    void setVelocityGain(double velocityGain);
    double getVelocityGain() const;
    

    void setPositionGain(double positionGain);
    double getPositionGain() const;
    
    void setFrequency(double frequency);
    double getFrequency() const;
        
private:
    
    double velocityGain;
    double positionGain;
    double frequency;
    
    sensor_msgs::JointState current_state;
    const ros::Publisher& armVelocitiesPublisher;
    const ros::Publisher& armPositionsPublisher;
    
private:
    
    double calculateVelocity(double actualAngle, 
        double actualVelocity, 
        const KDL::Trajectory_Composite& trajectoryComposite,
        double elapsedTimeInSec);
    
    void controlLoop(const std::vector<double>& actualJointAngles,
        const std::vector<double>& actualJointVelocities,
        const KDL::Trajectory_Composite* trajectoryComposite,
        int numberOfJoints,
        ros::Time startTime,
        std::vector<double>& velocities);
    
    void setTargetTrajectory(double angle1,
        double angle2,
        double duration,
        KDL::Trajectory_Composite& trajectoryComposite); 

};

#endif	/* JOINTTRAJECTORYACTION_H */

