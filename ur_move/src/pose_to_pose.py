#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MoveUR():

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group = "manipulator"
        self.arm_interface = moveit_commander.MoveGroupCommander(self.arm_group)
        self.waypoints = []

    def compute_path(self):
        current_ee_pose = self.arm_interface.get_current_pose().pose
        print("Current pose", current_ee_pose)
        pose_goal = copy.deepcopy(current_ee_pose)

        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        self.waypoints.append(copy.deepcopy(pose_goal))

        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.5
        pose_goal.position.y = -0.1
        pose_goal.position.z = 0.6

        self.waypoints.append(copy.deepcopy(pose_goal))

        print("WAYPOINTS", self.waypoints)

        (plan, fraction) = self.arm_interface.compute_cartesian_path(self.waypoints, 0.01, 0.0) 

        return plan

    def execute_plan(self,plan):
        execute_success = self.arm_interface.execute(plan)
        return execute_success

  
if __name__ == "__main__":
    rospy.init_node('moveit_manager')
    move_ur5 = MoveUR()
    path = move_ur5.compute_path()
    if path != None:
        move_ur5.execute_plan(path)
    else:
        print("NO PATH PLANNED!!!!!!!")

    rospy.spin()


