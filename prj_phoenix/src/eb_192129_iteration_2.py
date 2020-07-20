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


sorted_list = []
def goal_sort(goal_list):
    global sorted_list
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

def adjust_position():
    global regions
    speed = Twist()
    if (regions['front'] < d or regions['front'] < d or regions['front'] < d):

        speed.linear.x = 0
        pub_twist.publish(speed)
        #print(regions)
        print('face_obstacle')

    if (regions['front'] < d ):
        if regions['front'] < d and regions['fleft'] < d :
            #state_description = 'case 2 - front and left'
            for j in range(0,10):
                speed.linear.x = 0.5
                speed.angular.z = -0.2
                pub_twist.publish(speed)
    else:
        client.wait_for_server()



# val[3], goals_list=goals_list.sort(key = sortSecond, reverse = True)

# Initialize node
rospy.init_node("prj_phoenix")

# Subscribe to /goals topic to fetch the goal points
sub = rospy.Subscriber("/goals", PointArray, callback_goalpoint)

# Subscribe to /amcl_pose topic to localize the turtlebot in the given map
sub_loc = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped , callback_position)

# Creating an action client that communicates with action server /move_base that uses a message MoveBaseAction
<<<<<<< HEAD
client = actionlib.SimpleActionClient("move_base_simple/goal", MoveBaseAction)
# Action client waits for the action server to be launched and then sends the goals to the server
laser_sub = rospy.Subscriber("/scan" , LaserScan , callback_scan)
pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size =2)
=======
client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
# Action client waits for the action server to be launched and then sends the goals to the server
laser_sub = rospy.Subscriber("scan" , LaserScan , callback_scan)
pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size =2)
>>>>>>> b628529c479f0c3096161d1bb92e9e2311936ba6
client.wait_for_server()

rate = rospy.Rate(2)

while not goals_list:
    rospy.sleep(0.1)
list1 = goals_list
print(list1)


while not rospy.is_shutdown():

    n=len(goals_list)
    for i in range(n):
        #goals_list= sorted(goals_list , key = lambda x : x[2] , reverse = False)
        #sorted_list = goal_sort(goals_list)
        goal=goal_struct(goals_list[i])
        #a = goals_list.index(sorted_list[0])

        rospy.sleep(5)
        client.send_goal(goal)
        print(goal)
        client.wait_for_result(rospy.Duration(60))
        x=goal.target_pose.pose.position.x
        y=goal.target_pose.pose.position.y
        dist=math.sqrt((current_x_pos-x)**2 + (current_y_pos-y)**2)
        #if client.wait_for_result(rospy.Duration(5)):
            #print("Goal {} has skipped!".format(i))
            #client.get_state() == 4
            #client.cancel_goals_at_and_before_time()
            #adjust_position()


        if client.get_state() == 3 or dist<0.5:
            print("Goal {} has reached!".format(i))
            #del goals_list[i]
            rospy.sleep(1)
        else:
            #del goals_list[i]
            client.cancel_goal()
