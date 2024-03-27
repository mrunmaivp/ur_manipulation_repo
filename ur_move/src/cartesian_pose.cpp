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

const int Joints = 6;
KDL::JntArray jnt_pos_start(Joints);

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // Extract and print the current joint positions
    for (size_t i = 0; i < msg->position.size(); ++i) {
        ROS_INFO("Joint %zu position: %f", i, msg->position[i]);
    }
}

void get_goal_tcp_and_time(KDL::Frame tcp_pos_start, KDL::Vector* vec_tcp_pos_goal, float* t_max) {

	std::cout << "Please define the offset you want to move for each axis and the time in which the motion should be completed:\n";

		//Get user input
		float x,y,z;
		std::cout << "x:";
		std::cin >> x;
		std::cout << "y:";
		std::cin >> y;
		std::cout << "z:";
		std::cin >> z;
		std::cout << "Time:";
		std::cin >> (*t_max);

		//Compute goal position
		(*vec_tcp_pos_goal)(0) = (tcp_pos_start.p(0) + x);
		(*vec_tcp_pos_goal)(1) = (tcp_pos_start.p(1) + y);
		(*vec_tcp_pos_goal)(2) = (tcp_pos_start.p(2) + z);
}

float compute_linear(double q_start, double q_goal, float t, float t_max) {
	std::cout << "q_start: " << q_start << std::endl;
	std::cout << "q_goal: " << q_goal << std::endl;
	std::cout << "t: " << t << std::endl;
	std::cout << "t_max: " << t_max << std::endl;
	std::cout << "RESULT: " << ((q_goal - q_start) * (t/t_max) + q_start) << std::endl;
	return((q_goal - q_start) * (t/t_max) + q_start);
}

const int loop_rate_val = 100;

int main(int argc, char **argv){

    std::string urdf_path = ros::package::getPath("ur_description");
	if(urdf_path.empty()) {
		ROS_ERROR("ur_description package path was not found");
	}
	urdf_path += "/urdf/ur5_arm.urdf";
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

	//Generate a kinematic chain from the robot base to its tcp
	KDL::Chain ur5_chain;
	ur5_tree.getChain("base_link", "wrist_3_link", ur5_chain);

    KDL::ChainFkSolverPos_recursive fk_solver(ur5_chain);
	KDL::ChainIkSolverVel_pinv vel_ik_solver(ur5_chain, 0.0001, 1000);
	KDL::ChainIkSolverPos_NR ik_solver(ur5_chain, fk_solver, vel_ik_solver, 1000);

    ros::spinOnce(); // Process the initial joint states

	const float t_step = 1/((float)loop_rate_val);
	int count = 0;

	while (ros::ok()) {
        // Compute current tcp position
        KDL::Frame tcp_pos_start;
        fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);

        // Get user input for goal position and time
        float t_max;
        KDL::Vector vec_tcp_pos_goal(0.0, 0.0, 0.0);
        get_goal_tcp_and_time(tcp_pos_start, &vec_tcp_pos_goal, &t_max);
        KDL::Frame tcp_pos_goal(tcp_pos_start.M, vec_tcp_pos_goal);

        // Compute inverse kinematics
        KDL::JntArray jnt_pos_goal(Joints);
        ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal);

		float t = 0.0;
		while (t < t_max) {
			// Create a JointTrajectory message
			trajectory_msgs::JointTrajectory joint_trajectory_msg;
			joint_trajectory_msg.header.stamp = ros::Time::now();
			joint_trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

			// Compute next position step for all joints and add them to the trajectory message
			trajectory_msgs::JointTrajectoryPoint point;
			point.time_from_start = ros::Duration(t);
			for (int i = 0; i < Joints; i++) {
				point.positions.push_back(compute_linear(jnt_pos_start(i), jnt_pos_goal(i), t, t_max));
			}
			joint_trajectory_msg.points.push_back(point);

			// Publish the JointTrajectory message to joint_trajectory_controller/command topic
			joint_trajectory_pub.publish(joint_trajectory_msg);

			ros::spinOnce();
			loop_rate.sleep();
			++count;
			t += t_step;    
		}


        // Prepare and publish joint trajectory
        /*trajectory_msgs::JointTrajectory joint_trajectory_msg;
        joint_trajectory_msg.header.stamp = ros::Time::now();
        joint_trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        trajectory_msgs::JointTrajectoryPoint point;
        point.time_from_start = ros::Duration(t_max);

        // Populate trajectory point with desired joint positions
        for (int i = 0; i < Joints; ++i) {
            point.positions.push_back(compute_linear(jnt_pos_start(i), jnt_pos_goal(i), t_max, t_max));
			std::cout << i << "start" << jnt_pos_start(i) << std::endl;
			std::cout << i << "end" << jnt_pos_goal(i) << std::endl;
        }

        // Add trajectory point to the joint trajectory
        joint_trajectory_msg.points.push_back(point);

        // Publish the joint trajectory
        joint_trajectory_pub.publish(joint_trajectory_msg);

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();*/
    }
    return 0;

}
