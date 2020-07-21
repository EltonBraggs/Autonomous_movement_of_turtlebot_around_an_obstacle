#!/usr/bin/env python


import rospy
import math
from goal_publisher.msg import PointArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations
from tf.transformations import euler_from_quaternion
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseResult


from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
#from home.elton.catkin_ws.src.amr_multi_turtlebot.multi_turtlebot.launch import robots
#from launch import robots
d=0.5
pub_twist = None

regions={'right':0.0,
 'fright':0.0,
 'front':0.0,
 'fleft':0.0,
 'left':0.0
 }

def callback_scan(msg):
    global regions  ,   pub
    max_    =   3.5
    regions =   {'right'    :   min(min(msg.ranges[270:319]),max_),
     'fright'   :   min(min(msg.ranges[320:349]),max_),
     'front'    :   min(min(msg.ranges[0:10] , msg.ranges[350:360]),max_),
     'fleft'    :   min(min(msg.ranges[11:50]),max_),
     'left'     :   min(min(msg.ranges[51:90]),max_)
     }

# Defining a list - goals_list

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
	#adding goal coordinates and respective rewards to goals_list:
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

def adjust_position(current_x_pos , current_y_pos):
    global regions
    speed = Twist()
    while (regions['front'] > d and regions['fleft'] > d and regions['fright'] > d):

        speed.linear.x = 0
        pub_twist.publish(speed)
        #print(regions)
        print('face_obstacle')
        for j in range(0,10):
            speed.linear.x = 0.05
            speed.angular.z = -0.05
            pub_twist.publish(speed)

# Initialize node
rospy.init_node("prj_phoenix")

# Subscribe to /goals topic to fetch the goal points
sub = rospy.Subscriber("/goals", PointArray, callback_goalpoint)

# Subscribe to /amcl_pose topic to localize the turtlebot in the given map
sub_loc = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped , callback_position)

# Creating an action client that communicates with action server /move_base that uses a message MoveBaseAction
client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

# Action client waits for the action server to be launched and then sends the goals to the server

laser_sub = rospy.Subscriber("scan" , LaserScan , callback_scan)
pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size =2)
client.wait_for_server()

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    for i in range(len(final_order)):
        if final_order[i][3] > 9:
            goal = goal_struct(final_order[i])
            rospy.sleep(5)
            client.send_goal(goal)
            client.wait_for_result()
            if client.get_state() == 4 or client.get_state() == 5 or client.get_state() == 6 :
                print("Goal {} has skipped!".format(i))
                final_order.remove(final_order[i])
                adjust_position(current_x_pos, current_y_pos)
            if client.get_state() == 3:
                print("Goal {} has reached!".format(i) + " and the Reward is : " + str(final_order[i][3])) 
                rate.sleep()

             
