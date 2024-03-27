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

def move_group_python_interface_tutorial():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    print("Reference frame: %s" % move_group.get_planning_frame())

    print("End effector link: %s" % move_group.get_end_effector_link())

    print("Robot Groups:")
    print(robot.get_group_names())

    print("Current State:")
    print(robot.get_current_state())
    print("")

    # Planning to a Pose Goal
    print("Planning to a Pose Goal")

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Moving to a Joint Goal
    # print("Moving to a Joint Goal")

    # joint_goal = move_group.get_current_joint_values()
    # joint_goal[0] = -pi / 4
    # joint_goal[1] = -pi / 4
    # joint_goal[2] = -pi / 4
    # joint_goal[3] = -pi / 4
    # joint_goal[4] = -pi / 4
    # joint_goal[5] = -pi / 4
    # move_group.go(joint_goal, wait=True)
    # move_group.stop()

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_group_python_interface_tutorial()
    except rospy.ROSInterruptException:
        pass
