#!/usr/bin/env python

import rospy
# import sys 

# sys.path.append('/home/ros_noetic/ur_ws/src/')
# from motion_planning_python.srv import RobotState, RobotMove, RobotStateRequest, RobotStateResponse
from motion_generator_library.srv import ReadRobotState, ReadRobotStateRequest

def read_robot_state_client():
    rospy.wait_for_service('read_robot_state')
    try:
        read_robot_state_client = rospy.ServiceProxy('read_robot_state', RobotState)
        request = RobotStateRequest()
        request.read_robot_state = True
        response = read_robot_state_client(request)
        return response.joint_positions, response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None, False

if __name__ == "__main__":
    rospy.init_node('read_robot_state_client')
    joint_positions, success = read_robot_state_client()
    if success:
        print("Robot Joint Positions:", joint_positions)
    else:
        print("Failed to read robot state.")