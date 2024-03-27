#!/usr/bin/env python

import rospy
# import sys 

# sys.path.append('/home/ros_noetic/ur_ws/src/')
# from motion_planning_python.srv import RobotState, RobotMove, RobotStateRequest, RobotStateResponse, RobotMoveRequest
from motion_generator_library.srv import MoveRobot, MoveRobotRequest

def move_robot_client():
    rospy.wait_for_service('move_robot')
    try:
        joint_motion = True 
        cartesian_motion = False 
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
        # request.x = 10.5
        request.point1 = point1
        request.point2 = point2
        request.pose1 = pose1
        request.pose2 = pose2
        request.joint_velocity = joint_velocity
        request.joint_acceleration = joint_acceleration
        request.linear_velocity = linear_velocity
        request.linear_acceleration = linear_acceleration

        print("request.joint_motion ", request.joint_motion)
        print("request.cartesian_motion ", request.cartesian_motion)
        # print("request.x", request.x)
        print("request.point1 ", request.point1)
        print("request.point2 ", request.point2)
        print("request.pose1  ", request.pose1 )

        print("request.joint_motion ", request.joint_motion)
        response = move_robot_client(request)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

if __name__ == "__main__":
    rospy.init_node('move_robot_client')
    

    # joint_motion = True 
    # cartesian_motion = False 
    # point1 = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    # point2 = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7] 
    # pose1 = [0.5, 0.1, 0.3] 
    # pose2 = [0.5, 0.3, 0.5] 
    # joint_velocity = 0.1 
    # joint_acceleration = 0.05 
    # linear_velocity = 0.2 
    # linear_acceleration = 0.1 
    
 
    success = move_robot_client()
    
    if success:
        rospy.loginfo("Robot motion executed successfully.")
    else:
        rospy.logerr("Failed to execute robot motion.")
