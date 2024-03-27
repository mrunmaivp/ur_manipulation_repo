#!/usr/bin/env python

import rospy

from motion_generator_library.srv import ReadRobotState, ReadRobotStateRequest
from motion_generator_library.srv import MoveRobot, MoveRobotRequest

def read_robot_state_client():
    rospy.wait_for_service('read_robot_state')
    try:
        read_robot_state_client = rospy.ServiceProxy('read_robot_state', ReadRobotState)
        request = ReadRobotStateRequest()
        request.read_robot_state = True
        response = read_robot_state_client(request)
        return response.joint_positions, response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None, False



def move_robot_client():
    rospy.wait_for_service('move_robot')
    try:
        user_input = int(input("Do you want to manually input the values 0/1?"))
        if user_input == 1:
            joint_motion = input("Enter True or False for joint_motion: ")
            cartesian_motion = input("Enter True or False for cartesian_motion: ")


            point1 = [float(x) for x in input("Enter values for point1 (separated by space): ").split()]
            point2 = [float(x) for x in input("Enter values for point2 (separated by space): ").split()]

            pose1 = [float(x) for x in input("Enter values for pose1 (separated by space): ").split()]
            pose2 = [float(x) for x in input("Enter values for pose2 (separated by space): ").split()]

            joint_velocity = float(input("Enter joint_velocity: "))
            joint_acceleration = float(input("Enter joint_acceleration: "))
            linear_velocity = float(input("Enter linear_velocity: "))
            linear_acceleration = float(input("Enter linear_acceleration: "))
        else:
            joint_motion = False 
            cartesian_motion = True 
            point1 = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
            point2 = [-1.57, -1.0, 1.0, 1.0, 1.57, 0.0] 
            pose1 = [0.5, 0.1, 0.3] 
            pose2 = [0.5, 0.3, 0.5] 
            joint_velocity = 0.1 
            joint_acceleration = 0.05 
            linear_velocity = 0.2 
            linear_acceleration = 0.1

        move_robot_client = rospy.ServiceProxy('move_robot', MoveRobot)
        request = MoveRobotRequest()
        request.joint_motion = joint_motion
        request.cartesian_motion = cartesian_motion
        request.point1 = point1
        request.point2 = point2
        request.pose1 = pose1
        request.pose2 = pose2
        request.joint_velocity = joint_velocity
        request.joint_acceleration = joint_acceleration
        request.linear_velocity = linear_velocity
        request.linear_acceleration = linear_acceleration

        response = move_robot_client(request)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False




if __name__ == "__main__":
    rospy.init_node('read_robot_state_client')
    joint_positions, success = read_robot_state_client()
    print("Current Robot joint positions", joint_positions)
    robot_move_success = move_robot_client()
    print("Robot Move Sucess", robot_move_success)
    rospy.signal_shutdown('Shutting down the node')