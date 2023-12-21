#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
import threading
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np

# Initialize the node
rospy.init_node('robot_controller')

# MoveBaseClient class as before
class MoveBaseClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def move_to(self, x, y, qz, qw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = qw
        goal.target_pose.pose.orientation.z = qz

        self.client.send_goal(goal)
        self.client.wait_for_result()

        result = self.client.get_result()
        return result

# PoleFollower class as before
class PoleFollower:
    def __init__(self):
        rospy.loginfo('Pole Follower node initialized!')
        self.k_rho = 1
        self.k_alpha = 5
        self.r = 0.05
        self.test_angle = 15
        self.dif = 0.3
        self.pole_distance = 0.22

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def pole_detect(self):
        scan = rospy.wait_for_message('/scan', LaserScan)
        ranges = np.array(scan.ranges)
        ranges[ranges < 0.1] = 5000.0
        ranges = ranges.tolist()
        return self.find_cylinder(ranges)


    def find_cylinder(self, ranges):
        cylinders = []
        cylinders_index = []

        for i in range(1, 360):
            test_d = ranges[i]
            if (ranges[np.mod(i + 360 - self.test_angle, 360)] - test_d >= 0.2) \
                & (ranges[np.mod(i + self.test_angle, 360)] - test_d >= 0.2): 
                cylinders.append(ranges[i])
                cylinders_index.append(i)

        # if cylinders == ['empty']:
        if not cylinders:
            rospy.loginfo("No cylinder!")
            return 0, 0
        else:
            rho = min(cylinders)
            alpha = cylinders_index[cylinders.index(rho)]
            if alpha > 180:
                alpha = (alpha - 360) / 180 * np.pi
            else:
                alpha = alpha / 180 * np.pi
            return rho, alpha
    
    def move(self):
        rho, alpha = self.pole_detect()
        while rho > self.pole_distance:
            rho, alpha = self.pole_detect()
            # if rho <= 0.25:
            #     self.stop()
            #     rospy.sleep(2)
            #     break
            v = min(self.k_rho * rho, 0.2)
            omega = min(self.k_alpha * alpha, 2)

            twist = Twist()
            twist.linear.x = v
            twist.angular.z = omega
            self.cmd_vel_pub.publish(twist)
            
        self.stop()
        rospy.sleep(2)
        self.recover_back()
        rospy.sleep(3)

    def recover_back(self):
        twist = Twist()
        twist.linear.x = -0.09
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

# RobotController class to integrate MoveBaseClient and PoleFollower
class RobotController:
    def __init__(self):
        self.move_base_client = MoveBaseClient()
        self.pole_follower = PoleFollower()
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        rospy.sleep(2)
        self.set_initial_pose()

    def set_initial_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.seq = 10
        pose_msg.header.stamp = rospy.Time.now()  # 获取当前时间戳
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = -3.6882200241088867
        pose_msg.pose.pose.position.y = 4.894530773162842
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = -0.7151499283031105
        pose_msg.pose.pose.orientation.w = 0.6989710867039179
        pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        self.initial_pose_pub.publish(pose_msg)
        rospy.loginfo("InitialPose Set! ")

    def execute(self):
        positions = [
            (-3.751211, 4.280715, -0.438448, 0.898756),     # P1
            (-4.256124, 0.283748, -0.794614, 0.607115),     # P2
            (-0.244380, -0.086481, -0.710681, 0.703514),    # P3
            (0.093634, 3.932446, 0.698563, 0.715594)        # P4
        ]

        for index, position in enumerate(positions):
            # Move to position
            self.move_base_client.move_to(*position)
            rospy.loginfo("Goal P{} execution done! ".format(index + 1))
            rospy.sleep(1)

            # Detect pole
            if index >= 1:
                self.pole_follower.move()
                rospy.loginfo("Pole {} reached! ".format(index))

        # Return to P1
        self.move_base_client.move_to(*positions[0])
        rospy.loginfo("Back to P1 execution done!")

def main():
    controller = RobotController()
    controller.execute()

if __name__ == '__main__':
    main()
