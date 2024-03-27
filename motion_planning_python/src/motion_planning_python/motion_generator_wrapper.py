#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import sys 

sys.path.append('/home/ros_noetic/ur_ws/src/')
from motion_planning_python.srv import RobotState, RobotMove, RobotStateRequest, RobotStateResponse, RobotMoveRequest
from motion_planning_python.src.motion_planning_python import MotionGenerator

def joint_state_callback(msg):
    global current_joint_positions
    # print("Joint_state", msg)
    current_joint_positions = msg.position

def robot_state_callback(req):
    req = RobotStateRequest()
    print("REQUEST", req.read_robot_state)
    
    joint_positions = current_joint_positions
    success = True
    res = RobotStateResponse(joint_positions, success)
    
    return res

def robot_move_callback(req):
    rospy.loginfo("JOINT_MOTION %s", req.x)
    rospy.loginfo("JOINT_POSITION %s", req.point1)
    rospy.loginfo("CARTESIAN_MOTION %s", req.cartesian_motion)
    motion_generator = MotionGenerator()


if __name__ == "__main__":
    rospy.init_node("motion_planning_python_node")
    joint_states_subscriber = rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    robot_state_service = rospy.Service('robot_state', RobotState, robot_state_callback)
    robot_move_service = rospy.Service('robot_move', RobotMove, robot_move_callback)
    rospy.spin()
