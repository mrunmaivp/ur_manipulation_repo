#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "motion_generator_library/motion_generator_library.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_display_node");
    ros::NodeHandle nh;

    std::string type_of_motion = "cartesian";
    MotionGenerator motionGenerator;

    std::vector<double> jointPoint1 = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};
    std::vector<double> jointPoint2 = {-1.57, -1.0, 1.0, 1.0, 1.57, 0.0};
    double jointVelocity = 0.15;
    double jointAcceleration = 0.03;

    double linearVelocity = 0.1;
    double linearAcceleration = 0.01;

    geometry_msgs::Pose pose1, pose2;

    pose1.position.x = 0.5;
    pose1.position.y = 0.1;
    pose1.position.z = 0.2;
    pose1.orientation.w = 1.0;

    pose2.position.x = 0.5;
    pose2.position.y = 0.3;
    pose2.position.z = 0.5;
    pose2.orientation.w = 1.0;

    if (type_of_motion == "joint"){
        motionGenerator.joint_motion(jointPoint1, jointPoint2, jointVelocity, jointAcceleration);
    }
    else{
        motionGenerator.cartesian_motion(pose1, pose2, linearVelocity, linearAcceleration);
    }
    
    ros::spin();

    return 0;
}
