#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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
    try:
        # rospy.init_node('movebase_client_py')
        # movebase_client(0.3098, 4.0089, 0.0269, 0.9996)
        # rospy.loginfo("Goal P1 execution done!")
        # movebase_client(-4.2078, 0.3513, 0.9996, -0.0271)
        # rospy.loginfo("Goal P2 execution done!")
        # movebase_client(-0.1159, 0.0416, 0.6636, -0.7481)
        # rospy.loginfo("Goal P3 execution done!")
        # movebase_client(0.2838, 3.9930, 0.0264, 0.9996)
        # rospy.loginfo("Goal P4 execution done!")
        
        rospy.init_node('movebase_client_py')

    
        movebase_client(-4, 4, 1, 1)
        rospy.loginfo("Goal P1 execution done!")

        movebase_client(-4, 0, 1, 1)
        rospy.loginfo("Goal P2 execution done!")

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
