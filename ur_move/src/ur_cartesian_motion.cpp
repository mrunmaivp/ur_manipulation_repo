#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include <trajectory_msgs/JointTrajectory.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

sensor_msgs::JointStateConstPtr current_joint_state;

void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
    current_joint_state = msg;
}

int main(int argc, char **argv){

    std::string urdf_path = ros::package::getPath("ur_description");
    if(urdf_path.empty()) {
        ROS_ERROR("ur_description package path was not found");
        return false;
    }
    urdf_path += "/urdf/ur5_arm.urdf";

    std::cout << "urdf_path: " << urdf_path << std::endl;
    ros::init(argc, argv, "cartesian_pose_node");

    ros::NodeHandle nh;

    ros::Rate loop_rate(100);

    ros::Subscriber joint_states_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 10, jointStateCallback);

    ros::Publisher joint_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("eff_joint_traj_controller/command", 10);

    KDL::Tree ur5_tree;
    if (!kdl_parser::treeFromFile(urdf_path, ur5_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    KDL::Chain ur5_chain;
    ur5_tree.getChain("base_link", "wrist_3_link", ur5_chain);

    const double shoulder_pan_max = 6.28319;  
    const double shoulder_pan_min = -6.28319; 
    const double shoulder_lift_max = 6.28319;
    const double shoulder_lift_min = -6.28319;
    const double elbow_joint_max = 3.14159;   
    const double elbow_joint_min = -3.14159;
    const double wrist_1_max = 6.28319;
    const double wrist_1_min = -6.28319;
    const double wrist_2_max = 6.28319;
    const double wrist_2_min = -6.28319;
    const double wrist_3_max = 6.28319;
    const double wrist_3_min = -6.28319;

    KDL::JntArray q_min(6);
    KDL::JntArray q_max(6);


    q_min(0) = shoulder_pan_min;
    q_min(1) = shoulder_lift_min;
    q_min(2) = elbow_joint_min;
    q_min(3) = wrist_1_min;
    q_min(4) = wrist_2_min;
    q_min(5) = wrist_3_min;

    q_max(0) = shoulder_pan_max;
    q_max(1) = shoulder_lift_max;
    q_max(2) = elbow_joint_max;
    q_max(3) = wrist_1_max;
    q_max(4) = wrist_2_max;
    q_max(5) = wrist_3_max;

    KDL::ChainFkSolverPos_recursive fk_pos_solver(ur5_chain);
    KDL::ChainIkSolverVel_pinv ik_vel_solver(ur5_chain);
    KDL::ChainIkSolverPos_NR ik_solver(ur5_chain, fk_pos_solver, ik_vel_solver, 1000);

    KDL::JntArray joint_angles(ur5_chain.getNrOfJoints());

    while (ros::ok() && current_joint_state == nullptr) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    for (size_t i = 0; i < joint_angles.rows(); ++i) {
        const std::string& joint_name = ur5_chain.getSegment(i).getJoint().getName();
        std::cout << "Joint Name" << joint_name << std::endl;
        for (size_t j = 0; j < current_joint_state->name.size(); ++j) {
            if (current_joint_state->name[j] == joint_name) {
                joint_angles(i) = current_joint_state->position[j];
				std::cout << "Joint Angles" << joint_angles(i) << std::endl;
                break;
            }
        }
    }

	KDL::Frame end_effector_pose;
	int fk_result = fk_pos_solver.JntToCart(joint_angles, end_effector_pose);
	std::cout << "fk_result" << fk_result << std::endl;

    KDL::Vector position = end_effector_pose.p;
    std::cout << "End Effector Position (x, y, z): " << position.x() << ", " << position.y() << ", " << position.z() << std::endl;

    KDL::Rotation rotation = end_effector_pose.M;
    double roll, pitch, yaw;
    rotation.GetRPY(roll, pitch, yaw);
    std::cout << "End Effector Orientation (roll, pitch, yaw): " << roll << ", " << pitch << ", " << yaw << std::endl;

	KDL::JntArray required_joint_angles(6);
  	int ik_result = ik_solver.CartToJnt(joint_angles, end_effector_pose, required_joint_angles);
	std::cout << "ik_result" << ik_result << std::endl;

	for (int i = 0; i < required_joint_angles.rows(); ++i) {
    	std::cout << "Joint " << i << " angle: " << required_joint_angles(i) << std::endl;
	}

    KDL::Vector new_position(0.5, 0.1, 0.3); 
    KDL::Rotation new_orientation = KDL::Rotation::RPY(0.0, 0.0, 0.0); 
    KDL::Frame new_end_effector_pose(new_orientation, new_position);


    KDL::JntArray new_required_joint_angles(6);
    int ik_result_new = ik_solver.CartToJnt(joint_angles, new_end_effector_pose, new_required_joint_angles);
    if (ik_result_new != KDL::ChainIkSolverPos_NR::E_NOERROR) {
        ROS_ERROR("Failed to find IK solution for the desired pose");
        return 1;
    }
    for (int i = 0; i < new_required_joint_angles.rows(); ++i) {
    	std::cout << "New Joint " << i << " angle: " << new_required_joint_angles(i) << std::endl;
	}

    trajectory_msgs::JointTrajectory joint_trajectory_msg;
    joint_trajectory_msg.header.stamp = ros::Time::now();
    joint_trajectory_msg.joint_names = current_joint_state->name; 

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(new_required_joint_angles.rows());
    for (size_t i = 0; i < new_required_joint_angles.rows(); ++i) {
        point.positions[i] = new_required_joint_angles(i);
    }
    point.time_from_start = ros::Duration(1.0);  

    joint_trajectory_msg.points.push_back(point);

    joint_trajectory_pub.publish(joint_trajectory_msg);

    ros::spinOnce();
    loop_rate.sleep();

    return 0;
}
