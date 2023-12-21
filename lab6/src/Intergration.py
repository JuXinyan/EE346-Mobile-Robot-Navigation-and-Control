#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class PoleFollower:
    def __init__(self):
        rospy.loginfo('Pole Follower node initialized!')
        self.k_rho = 1
        self.k_alpha = 5

        self.r = 0.05
        self.test_angle = 15
        self.dif = 0.3

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


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
            if (ranges[np.mod(i + 360 - self.test_angle, 360)] - test_d >= 0.3) \
                & (ranges[np.mod(i + self.test_angle, 360)] - test_d >= 0.3): 
                cylinders.append(ranges[i])
                cylinders_index.append(i)

        if cylinders == ['empty']:
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
        while not rospy.is_shutdown():
            rho, alpha = self.pole_detect()
            if rho <= 0.25:
                self.stop()
                continue

            v = min(self.k_rho * rho, 0.2)
            omega = min(self.k_alpha * alpha, 2)

            twist = Twist()
            twist.linear.x = v
            twist.angular.z = omega

            self.pub.publish(twist)



    def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub.publish(twist)

def movebase_client(x, y, q_w, q_z):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.pose.orientation.z = q_z

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    rospy.init_node('Intergration')
    try:
        pole_follower = PoleFollower()
    
        movebase_client(-4, 4, 1, 1)
        rospy.loginfo("Goal P1 execution done!")

        movebase_client(-4, 0, 1, 1)
        rospy.loginfo("Goal P2 execution done!")
        pole_follower.move()

        movebase_client(0, 0, 1, 1)
        rospy.loginfo("Goal P3 execution done!")



        movebase_client(0, 4, 0.9996, -0.0271)
        rospy.loginfo("Goal P4 execution done!")



        movebase_client(-4, 4, -0.7106, 0.7036)
        rospy.loginfo("Back P1 execution done!")

        # if result:
        #     rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")







