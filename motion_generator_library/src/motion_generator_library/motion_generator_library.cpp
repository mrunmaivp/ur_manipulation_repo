#include "motion_generator_library/motion_generator_library.h"

MotionGenerator::MotionGenerator() :
    arm_group("manipulator"),
    client("/eff_joint_traj_controller/follow_joint_trajectory", true),
    joint_names({"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}) {
    int argc;
    char** argv;
    ros::init(argc, argv, "motion_generator");
}

void MotionGenerator::joint_motion(std::vector<double> point1, std::vector<double> point2,
                                   double joint_velocity, double joint_acceleration) {
    client.waitForServer();
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory &trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;

    trajectory_msgs::JointTrajectoryPoint point;

    point.positions = point1;
    point.velocities = std::vector<double>(6, joint_velocity);
    point.accelerations = std::vector<double>(6, joint_acceleration);
    point.time_from_start = ros::Duration(5.0);
    trajectory.points.push_back(point);

    point.positions = point2;
    point.velocities = std::vector<double>(6, 0.0); 
    point.accelerations = std::vector<double>(6, 0.04); 
    point.time_from_start = ros::Duration(10.0); 
    trajectory.points.push_back(point);

    client.sendGoalAndWait(goal);
}

bool MotionGenerator::cartesian_motion(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2,
                                       double linear_velocity, double linear_acceleration) {


    double fraction;
    int trials = 0;
    //while(fraction != 1.0 && trials < 2){
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(pose1);
    waypoints.push_back(pose2);

    double max_velocity_scaling_factor = linear_velocity;
    double max_acceleration_scaling_factor = linear_acceleration;

    arm_group.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    arm_group.setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
    moveit_msgs::RobotTrajectory trajectory;
    fraction = arm_group.computeCartesianPath(waypoints, 0.001, 0.0, trajectory);
    arm_group.execute(trajectory);
    if (fraction < 1){
        return false;
    }
    else{
        return true;
    }
    //trials += 1;

    //}
}


