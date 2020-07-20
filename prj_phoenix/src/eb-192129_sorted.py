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
final_order = list()
current_y_pos = 0
current_x_pos = 0

# Function to calculate distance
def dist_calc(currx, curry, goalx, goaly):
    dist = math.sqrt((goalx - currx) ** 2 + (goaly - curry) ** 2)
    return dist

# Callback function for the /goals topic Subscriber
def callback_goalpoint(msg):
    global goals_list
    global current_x_pos
    global current_y_pos
    initx = current_x_pos
    inity = current_y_pos
    for i in range(len(msg.goals)):
        goals_list.append([msg.goals[i].x, msg.goals[i].y, msg.goals[i].z, msg.goals[i].reward])

    for j in range(len(msg.goals)):
        dist_list = []
        for k in range(len(goals_list)):
            dist_list.append(dist_calc(initx, inity, msg.goals[k].x, msg.goals[k].y))
        final_order.append(goals_list[dist_list.index(min(dist_list))])
        initx = goals_list[dist_list.index(min(dist_list))][0]
        inity = goals_list[dist_list.index(min(dist_list))][1]
        goals_list.pop(dist_list.index(min(dist_list)))

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
    my_goals.target_pose.pose.position.x = point[0]
    my_goals.target_pose.pose.position.y = point[1]
    my_goals.target_pose.pose.position.z = 0
    my_goals.target_pose.pose.orientation.x = 0
    my_goals.target_pose.pose.orientation.y = 0
    my_goals.target_pose.pose.orientation.z = 0
    my_goals.target_pose.pose.orientation.w = 1
    return my_goals


# Initialize node
rospy.init_node("prj_phoenix")

# Subscribe to /goals topic to fetch the goal points
sub = rospy.Subscriber("/goals", PointArray, callback_goalpoint)

# Subscribe to /amcl_pose topic to localize the turtlebot in the given map
sub_loc = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped , callback_position)

# Creating an action client that communicates with action server /move_base that uses a message MoveBaseAction
client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
# Action client waits for the action server to be launched and then sends the goals to the server
client.wait_for_server()

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    for i in range(len(final_order)):
        if final_order[i][3] > 9:
            goal = goal_struct(final_order[i])
            rospy.sleep(5)
            client.send_goal(goal)
            client.wait_for_result()
            if client.get_state() == 3:
                print("Goal {} has reached!".format(i))
                rate.sleep()
