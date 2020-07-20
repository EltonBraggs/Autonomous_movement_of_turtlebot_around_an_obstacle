#!/usr/bin/env python

import rospy
import math
from goal_publisher.msg import PointArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations
from tf.transformations import euler_from_quaternion
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseResult

# Defining a list - goals_list
goals_list = list()

# Callback function for the /goals topic Subscriber
def callback_goalpoint(msg):
    global goals_list
    for i in range(len(msg.goals)):
        if msg.goals[i].reward > 0:
        	goals_list.append([[msg.goals[i].x, msg.goals[i].y, msg.goals[i].z,msg.goals[i].reward]])


def callback_position(pos):
    global current_x_pos
    global current_y_pos
    global current_theta
    current_x_pos = pos.pose.pose.position.x
    current_y_pos = pos.pose.pose.position.y
    rot_q=pos.pose.pose.orientation
    (roll,pitch,current_theta)=euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])


# Crearting the goal message and sending it to the action server
def goal_struct(point):
    my_goals = MoveBaseGoal()
    my_goals.target_pose.header.frame_id = "map"
    my_goals.target_pose.pose.position.x = point[0][0]
    my_goals.target_pose.pose.position.y = point[0][1]
    my_goals.target_pose.pose.position.z = 0
    my_goals.target_pose.pose.orientation.x = 0
    my_goals.target_pose.pose.orientation.y = 0
    my_goals.target_pose.pose.orientation.z = 0
    my_goals.target_pose.pose.orientation.w = 1
    return my_goals






# val[3], goals_list=goals_list.sort(key = sortSecond, reverse = True)

# Initialize node
rospy.init_node("final_prj")

# Subscribe to /goals topic to fetch the goal points
sub = rospy.Subscriber("/goals", PointArray, callback_goalpoint)

# Subscribe to /amcl_pose topic to localize the turtlebot in the given map
sub_loc = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped , callback_position)

# Creating an action client that communicates with action server /move_base that uses a message MoveBaseAction
client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
# Action client waits for the action server to be launched and then sends the goals to the server
client.wait_for_server()

rate = rospy.Rate(2)

while not rospy.is_shutdown():
    n=len(goals_list)
    for i in range(n):
        #goals_list= goal_sort()
        goal=goal_struct(goals_list[1])

        rospy.sleep(5)
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(2))
        x=goal.target_pose.pose.position.x
        y=goal.target_pose.pose.position.y
        d=math.sqrt((current_x_pos-x)**2 + (current_y_pos-y)**2)


        if client.get_state() == 3 or d<0.015:
            print("Goal {} has reached!".format(i))
            rospy.sleep(1)
            del goals_list[i]
        else:
            client.cancel_goal()
