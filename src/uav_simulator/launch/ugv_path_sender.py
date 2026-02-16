#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('send_goal')
pub = rospy.Publisher('/scout1/move_base/goal', PoseStamped, queue_size=1)

goal = PoseStamped()
goal.header.frame_id = "map"
goal.pose.position.x = 2.0
goal.pose.position.y = 1.0
goal.pose.orientation.z = 0.7071
goal.pose.orientation.w = 0.7071

rospy.sleep(1)
pub.publish(goal)
print("Goal sent!")

