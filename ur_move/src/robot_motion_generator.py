
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class RobotMotionGenerator():
    def __init__(self, type_of_motion):
        rospy.init_node('motion_generator')
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
        self.type = type_of_motion
        self.client = actionlib.SimpleActionClient('/eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    def joint_motion(self, point1, point2, joint_velocity, joint_acceleration):
        self.client.wait_for_server()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = np.array(point1) 
        point.velocities = [0.15] * 6  
        point.accelerations = [0.03] * 6  
        point.time_from_start = rospy.Duration(5.0)
        goal.trajectory.points.append(point)

        point = JointTrajectoryPoint()
        point.positions = np.array(point2)
        point.velocities = [0.0] * 6  # Ending point velocities
        point.accelerations = [0.04] * 6  # Ending point accelerations
        point.time_from_start = rospy.Duration(10.0)  # Time for the ending point
        goal.trajectory.points.append(point)

        self.joint_client.send_goal_and_wait(goal)

    def cartesian_motion(self, pose1, pose2, linear_velocity, linear_accleration):
        waypoints = [pose1, pose2]
        print("WAYPOINTS", waypoints)

        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)

        if plan != 1.0:
            self.arm_group.execute(plan)
        else:
            rospy.logerr("Failed to generate valid Cartesian path")


