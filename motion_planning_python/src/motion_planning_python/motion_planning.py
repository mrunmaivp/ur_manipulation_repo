#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import Pose
from actionlib import SimpleActionClient
from moveit_commander import MoveGroupCommander

class MotionGenerator():
    def __init__(self):
        rospy.init_node("motion_generator")
        self.arm_group = MoveGroupCommander("manipulator")
        self.client = SimpleActionClient('/eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.client.wait_for_server()

    def joint_motion(self, point1, point2, joint_velocity, joint_acceleration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = point1
        point.velocities = [joint_velocity] * 6
        point.accelerations = [joint_acceleration] * 6
        point.time_from_start = rospy.Duration(5.0)
        goal.trajectory.points.append(point)

        point = JointTrajectoryPoint()
        point.positions = point2
        point.velocities = [0.0] * 6
        point.accelerations = [0.04] * 6
        point.time_from_start = rospy.Duration(10.0)
        goal.trajectory.points.append(point)

        self.client.send_goal_and_wait(goal)

    def cartesian_motion(self, pose1, pose2, linear_velocity, linear_acceleration):
        waypoints = [pose1, pose2]
        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        rospy.loginfo("Computed path fraction: %.2f", fraction)
        if plan != None:
            # plan = self.arm_group.retime_trajectory(self.arm_group.get_current_state(), trajectory, linear_velocity)
            self.arm_group.execute(plan)
        else:
            rospy.logerr("Failed to generate valid Cartesian path")