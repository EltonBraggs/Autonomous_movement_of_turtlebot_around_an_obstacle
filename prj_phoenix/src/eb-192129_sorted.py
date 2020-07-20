#!/usr/bin/env python

import rospy
import math
from goal_publisher.msg import PointArray 
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations 
from tf.transformations import euler_from_quaternion
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseResult

# Defining list - goals_list[containing goals and rewards], final_order[ascending goal list on the basis of distance betwen robot & goal]
goals_list = list()
final_order = list()
current_y_pos = 0
current_x_pos = 0

# Function to calculate distance between goal and current position of robot:
def dist_calc(currx, curry, goalx, goaly):
    dist = math.sqrt((goalx - currx) ** 2 + (goaly - curry) ** 2)
    return dist

# Callback function for the /goals topic Subscriber:
def callback_goalpoint(msg):
    global goals_list
    global current_x_pos
    global current_y_pos
    initx = current_x_pos
    inity = current_y_pos
    for i in range(len(msg.goals)):
	#adding goal coordinates and respective rewards to goals_list
        goals_list.append([msg.goals[i].x, msg.goals[i].y, msg.goals[i].z, msg.goals[i].reward]) 

    #appending dist_list and final_order lists: 
    for j in range(len(msg.goals)):
        dist_list = []
        for k in range(len(goals_list)):
            dist_list.append(dist_calc(initx, inity, msg.goals[k].x, msg.goals[k].y)) #adding distance to list by calling dist_calc function
        final_order.append(goals_list[dist_list.index(min(dist_list))]) #modifying goal list on basis of ascending distance
        initx = goals_list[dist_list.index(min(dist_list))][0] #x coordinate of goal at minimum distance
        inity = goals_list[dist_list.index(min(dist_list))][1] #y coordinate of goal at minimum distance
        goals_list.pop(dist_list.index(min(dist_list))) #removing goal at current minimum distance after reaching at goal

# callback function to octain present x and y coordinates, orientation of the turtlebot
def callback_position(pos): 
    global current_x_pos
    global current_y_pos
    global current_theta
    current_x_pos = pos.pose.pose.position.x
    current_y_pos = pos.pose.pose.position.y
    rot_q=pos.pose.pose.orientation
    (roll,pitch,current_theta)=euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])


# Creating the goal message and sending it to the action server
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
sub_loc = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped , callback_position)

# Creating an action client that communicates with action server /move_base that uses a message MoveBaseAction
client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

# Action client waits for the action server to be launched and then sends the goals to the server
client.wait_for_server()

rate = rospy.Rate(1)

while not rospy.is_shutdown(): #code run until interrupted
    for i in range(len(final_order)):
        if final_order[i][3] > 0: 
            goal = goal_struct(final_order[i]) #defining next nearest goal from final_order list
            rospy.sleep(5)
            client.send_goal(goal) #sending goal coordinates
            client.wait_for_result()
            if client.get_state() == 3: #when acheived state 3 - goal acheived successfully by robot  
                print("Goal {} has reached!".format(i) + " and the Reward is : " + str(final_order[i][3])) 
		#if self.i == 9:
                #    print "all points are reached"
                rate.sleep()
