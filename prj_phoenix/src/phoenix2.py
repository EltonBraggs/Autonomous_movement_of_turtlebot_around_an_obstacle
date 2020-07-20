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
        goals_list.append([msg.goals[i].x, msg.goals[i].y,msg.goals[i].reward])


def callback_position(pos):
    global current_x_pos
    global current_y_pos
    global current_theta
    current_x_pos = pos.pose.pose.position.x
    current_y_pos = pos.pose.pose.position.y
    rot_q=pos.pose.pose.orientation
    (roll,pitch,current_theta)=euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

# Initialize node
rospy.init_node("prj_phoenix")

# Subscribe to /goals topic to fetch the goal points
sub = rospy.Subscriber("/goals", PointArray, callback_goalpoint)

# Subscribe to /amcl_pose topic to localize the turtlebot in the given map
sub_loc = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped , callback_position)
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        rospy.loginfo(str(points))
        for point in points:
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        #rospy.loginfo(str(self.pose_seq))
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        #wait = self.client.wait_for_server(rospy.Duration(5.0))
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server")

    def feedback_cb(self, feedback):
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
    
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return
  
# Crearting the goal message and sending it to the action server
def goal_struct(point):
    my_goals = MoveBaseGoal()
    my_goals.target_pose.header.frame_id = "map"
    my_goals.target_pose.pose = self.pose_seq[self.goal_cnt]
    my_goals.target_pose.pose.position.x = point[0]
    my_goals.target_pose.pose.position.y = point[1]
    my_goals.target_pose.pose.position.z = 0
    my_goals.target_pose.pose.orientation.x = 0
    my_goals.target_pose.pose.orientation.y = 0
    my_goals.target_pose.pose.orientation.z = 0
    my_goals.target_pose.pose.orientation.w = 1
    rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
    rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
    self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
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
