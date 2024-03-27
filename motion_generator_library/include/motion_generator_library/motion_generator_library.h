
#ifndef MOTION_GENERATOR_LIBRARY_H
#define MOTION_GENERATOR_LIBRARY_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class MotionGenerator {
public:
    MotionGenerator();

    void joint_motion(std::vector<double> point1, std::vector<double> point2,
                      double joint_velocity, double joint_acceleration);

    bool cartesian_motion(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2,
                          double linear_velocity, double linear_acceleration);

private:
    moveit::planning_interface::MoveGroupInterface arm_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client;
    std::vector<std::string> joint_names;
};

#endif
