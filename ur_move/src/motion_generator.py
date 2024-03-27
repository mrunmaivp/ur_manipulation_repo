#!/usr/bin/env python

import rospy
import sys
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
import copy
from geometry_msgs.msg import Pose

class MotionGenerator:
    def __init__(self):
        rospy.init_node('motion_generator')
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
        self.joint_client = actionlib.SimpleActionClient('/eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        

    def generate_joint_motion(self, point1, point2, velocity, acceleration):
        self.joint_client.wait_for_server()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        point = JointTrajectoryPoint()
        point.positions = point1
        point.velocities = [velocity] * 6
        point.accelerations = [acceleration] * 6
        point.time_from_start = rospy.Duration(5.0)  # Time for the starting point
        goal.trajectory.points.append(point)

        point = JointTrajectoryPoint()
        point.positions = point2
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.time_from_start = rospy.Duration(10.0)  # Time for the ending point
        goal.trajectory.points.append(point)

        self.joint_client.send_goal_and_wait(goal)

    def generate_linear_motion(self, pose1, pose2, linear_velocity, linear_acceleration):
        waypoints = [pose1, pose2]
        print("WAYPOINTS", waypoints)

        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)

        if plan != 1.0:
            self.arm_group.execute(plan)
        else:
            rospy.logerr("Failed to generate valid Cartesian path")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        motion_generator = MotionGenerator()

        # Example inputs
        point1 = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        point2 = [-1.57, -1.0, 1.0, 1.0, 1.57, 0.0]

        pose1 = Pose()
        pose1.position.x = 0.4
        pose1.position.y = 0.1
        pose1.position.z = 0.4
        pose1.orientation.w = 1.0

        pose2 = Pose()
        pose2.position.x = 0.5
        pose2.position.y = -0.1
        pose2.position.z = 0.6
        pose2.orientation.w = 1.0

        # motion_generator.generate_joint_motion(point1, point2, velocity=0.15, acceleration=0.03)
        motion_generator.generate_linear_motion(pose1, pose2, linear_velocity=0.01, linear_acceleration=0.0)
        
        motion_generator.spin()

    except rospy.ROSInterruptException:
        pass
