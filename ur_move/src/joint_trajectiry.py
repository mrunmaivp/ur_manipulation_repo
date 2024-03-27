#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_trajectory():
    rospy.init_node('ur5_trajectory_sender', anonymous=True)

    client = actionlib.SimpleActionClient('/eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]  # Starting point positions
    point.velocities = [0.15] * 6  # Starting point velocities
    point.accelerations = [0.03] * 6  # Starting point accelerations
    point.time_from_start = rospy.Duration(5.0)  # Time for the starting point
    goal.trajectory.points.append(point)

    point = JointTrajectoryPoint()
    point.positions = [-1.57, -1.0, 1.0, 1.0, 1.57, 0.0]  # Ending point positions
    point.velocities = [0.0] * 6  # Ending point velocities
    point.accelerations = [0.04] * 6  # Ending point accelerations
    point.time_from_start = rospy.Duration(10.0)  # Time for the ending point
    goal.trajectory.points.append(point)
    
    # point = JointTrajectoryPoint()
    # point.positions = [1.57, 1.57, -1.0, -1.0, -1.57, 0.0]  # Ending point positions
    # point.velocities = [0.0] * 6  # Ending point velocities
    # point.accelerations = [0.04] * 6  # Ending point accelerations
    # point.time_from_start = rospy.Duration(15.0)  # Time for the ending point
    # goal.trajectory.points.append(point)

    print("GOAL", goal)
    client.send_goal(goal)
    client.wait_for_result()

if __name__ == '__main__':
    try:
        send_trajectory()
    except rospy.ROSInterruptException:
        pass
