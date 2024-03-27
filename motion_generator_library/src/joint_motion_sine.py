#! /usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

def publish_joint_states():
    rospy.init_node('ur_publish_joint_states')

    joint_state_publisher = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    while not rospy.is_shutdown():
	
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        point = JointTrajectoryPoint()
        time = rospy.Time.now().to_sec()

        j1 = math.sin(time * 0.1)
        j2 = math.sin(time * 0.2)
        j3 = math.sin(time * 0.3)
        j4 = math.sin(time * 0.4)
        j5 = math.sin(time * 0.5)
        j6 = math.sin(time * 0.6)

        point.positions = [j1, j2, j3, j4, j5, j6]
        point.velocities = [0, 0, 0, 0, 0, 0]  # Set velocities to zero for simplicity
        point.accelerations = [0, 0, 0, 0, 0, 0]  # Set accelerations to zero for simplicity
        point.effort = []

        point.time_from_start = rospy.Duration(1)

        msg.points.append(point)

        joint_state_publisher.publish(msg)

        rospy.loginfo(msg)

	
if __name__ == '__main__':
    publish_joint_states()
