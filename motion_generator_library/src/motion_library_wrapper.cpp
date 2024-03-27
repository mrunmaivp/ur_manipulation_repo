#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "motion_generator_library/motion_generator_library.h"
#include "motion_generator_library/ReadRobotState.h"
#include "motion_generator_library/MoveRobot.h"
#include "sensor_msgs/JointState.h"

std::vector<double> current_joint_positions;

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    current_joint_positions = msg->position;
}

bool readRobotStateCallback(motion_generator_library::ReadRobotState::Request& req,
                            motion_generator_library::ReadRobotState::Response& res) {
    
    std::cout << "ROBOT STATE" << req.read_robot_state << std::endl;
    
    res.joint_positions = current_joint_positions;
    res.success = true;
    return true;
}

bool moveRobotCallback(motion_generator_library::MoveRobot::Request& req,
                       motion_generator_library::MoveRobot::Response& res) {


    MotionGenerator motionGenerator;

    bool success = false;
    bool motion_success;
    if (req.joint_motion) {
        motionGenerator.joint_motion(req.point1, req.point2, req.joint_velocity, req.joint_acceleration);
        success = true;
    }
    if (req.cartesian_motion) {
        geometry_msgs::Pose pose1, pose2;
        pose1.position.x = req.pose1[0];
        pose1.position.y = req.pose1[1];
        pose1.position.z = req.pose1[2];
        pose1.orientation.w = 1.0;
        pose2.position.x = req.pose2[0];
        pose2.position.y = req.pose2[1];
        pose2.position.z = req.pose2[2];
        pose2.orientation.w = 1.0;
        motion_success = motionGenerator.cartesian_motion(pose1, pose2, req.linear_velocity, req.linear_acceleration);
        success = motion_success;
    }

    res.success = success;
    return true;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_wrapper_node");

    ros::NodeHandle nh;
    ros::Subscriber joint_states_subscriber = nh.subscribe("/joint_states", 1, jointStatesCallback);
    ros::ServiceServer read_robot_state_service = nh.advertiseService("read_robot_state", readRobotStateCallback);
    ros::ServiceServer move_robot_service = nh.advertiseService("move_robot", moveRobotCallback);


    ros::spin();

    return 0;
}
