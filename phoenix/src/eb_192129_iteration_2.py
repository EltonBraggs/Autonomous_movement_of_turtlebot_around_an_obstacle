#!/usr/bin/env python

import rospy
import math
from goal_publisher.msg import PointArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations
from tf.transformations import euler_from_quaternion
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseResult
#from home.elton.catkin_ws.src.amr_multi_turtlebot.multi_turtlebot.launch import robots
#from launch import robots




# Defining a list - goals_list
goals_list = list()

# Callback function for the /goals topic Subscriber
def callback_goalpoint(msg):
    global goals_list
    for i in range(len(msg.goals)):
        goals_list.append([msg.goals[i].x, msg.goals[i].y,msg.goals[i].reward])


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



def goal_sort(goal_list):

    dist=[]
    sorted_list = []

    for i in range(len(goal_list)):

        x=goal_list[i][0]
        y=goal_list[i][1]
        d=math.sqrt((current_x_pos-x)**2 + (current_y_pos-y)**2)
        norm = goal_list[i][2]/d
        goal_list[i].insert(3, norm)
        #goal_list[i][3].append(norm)

    sorted_list= sorted(goal_list , key = lambda x : x[3] , reverse = True)
    print(sorted_list)
    return sorted_list






# val[3], goals_list=goals_list.sort(key = sortSecond, reverse = True)

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

rate = rospy.Rate(2)

while not goals_list:
    rospy.sleep(0.1)
list1 = goals_list
print(list1)


while not rospy.is_shutdown():

    n=len(goals_list)
    for i in range(n):
        sorted_list = goal_sort(goals_list)
        goal=goal_struct(sorted_list[0][:])
        a = goals_list.index(sorted_list[0])

        rospy.sleep(5)
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(60))
        x=goal.target_pose.pose.position.x
        y=goal.target_pose.pose.position.y
        d=math.sqrt((current_x_pos-x)**2 + (current_y_pos-y)**2)


        if client.get_state() == 3 or d<0.015:
            print("Goal {} has reached!".format(i))
            goals_list.remove(goals_list[a])
            rospy.sleep(1)
        else:
            client.cancel_goal()
