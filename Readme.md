#Tier 4

AMR Final Project :\
**Team Name : Phoenix**\
Members :
1. Rishabh Rana @rr-192152
2. Elton Wilroy Braggs @eb-192129

# Project Description:
# To move robot to specified locations by avoiding obstacles and other robots in arena:

**Robot used for simulation** : turtlebot3 with 360 degree laser scanner, scan for obstacles in path.\
**Software for simulation** : Gazebo.\

There are total 10 goals which are desired locations for robot. In the path, there are various obstacles eg. blocks, cones, maze, wall.
Goals coordinates will be provided. These goal points will be imported to take the goals coordinates data from it and use in this package. Robot moves to defined 10 goals and collecting reward points, avoiding collisions with obstacles and other three robots.

Packages details to launch this programm :\
- **package**     : prj_phoenix
- **code file**   : final_prj_phoenix.py
- **launch file** : start_final.launch

# Launch file :
The robot will start using the launch file = start_final.launch. The node in this launch file is 'prj_phoenix'.\
There will be four robots at a time in the arena and register to same ros master and goal publisher, therefore a concept 'namespace' is used in this file, to avoid conflicts among the four robots.\
For that an argument is passed in launch file for robot number :  <arg name="robot_id" default="robot1"/>.
Thus, there will be a different set of topics for each robot, where they are subscribed. 

# Steps in the simulation :-
**For testing robot in practice arena:**
1. Launch multi-turtlebot package with main.launch file, this will launch a practice arena with given model and map. For model and map file, arguments for absolute path must be given. This can be launched with following command:
```
roslaunch multi_turtlebot main.launch model_file:=/abolute_path/to/arena.sdf map_file:=/path/to/map2.yaml
```
2. Now launch the goal publisher file, to fetch the goals' data by launching goal_publisher with practice_goals2.yaml config file.
```
roslaunch goal_publisher goal_publisher.launch config_file:=practice_goals2.yaml
```
3. Launch the package prj_phoenix with start_final.launch file and argument for robot_id. Now the robot will subscribe to one set of data  for goals and start moving towards the goals, avoiding the obstacles and other robots.
   The position of bot and goal number will be published on the terminal.
```
roslaunch prj_phoenix start_final.launch robot_id:= robot1
```
The argument value of robot_id can be changed.

# General Description of the Project

Included Topics:
1. **/goals** - All goal points are published into this topic and it is subscribed it to obtain the goals.
2. **amcl_pos** - To localize the turtlebot in the given map. [geometry_msgs/PoseWithCovarianceStamped]
package:\
**move_base package** : In this project, we used an action client that communicates with action server /move_base that uses a message MoveBaseAction.\
   client = actionlib.SimpleActionClient("move_base", MoveBaseAction)\
It provides an implementation of an action that, given a goal in the world, will attempt to reach it with a mobile base. 
The move_base action node links together a global and local planner to accomplish its global navigation task.


# Algorithm Description:
Firstly, the robot will subscribe to a personal set of topics and fetch goals data. Then a list will be created to store this goals data, 'goals_list'. The program will then calculate the distance between the goals and the current position of robot and store these values in a list called as 'dist'. Then an ascending list will be created on the basis of distance data, 'final-order', and the robot will move to the nearest goal first as per this list and collect the respective reward. After reaching at the goal with a tolerance of 0.5, a message will be printed on the terminal indicating reached goal number and reward value. Then, robot will move to next goal as per the final list.
\
The feedback server is required to provide the following information:\
	If the goal status is active, it creates a local path from the current position to the goal position. Then it subscribes to the cmd_vel topic for the turtlebot to move.\
If the goal status from the feedback server is 4 , 5 or 6 , it skips the goal and moves to the next goal. 

**Parameter values changed from the default move base parameters:**
>Path_distance_bias = 8.0\
>Inflation_radius:\
>>global_costmap = 0.2\
>>local_costmap = 0.1\
>Velocity and the width values were increased.


