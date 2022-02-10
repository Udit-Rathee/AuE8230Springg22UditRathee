READ ME.md
This repositry contains solution to Assignment 2 where turtle bot simulations were carried out via python scripts.

Heirarchy here: 

assignment_workspace: the main catkin workspace

assignment_workspace/src/assignment2_turtlesim: the package build in the catkin workspace 

assignment_workspace/src/assignment2_turtlesim/src: Contains the following python scripts:

1. circle.py 
This script makes the bot traverse a circle of radius 2 unit at a constant twist velocity of 1 rad/sec. The spawn location acts as a point on the circumference of this circle 

2. square_open.py 
This script makes the bot traverse a square of side 2 unit at a constant linear and angular velocity of 0.2 unit/sec and 0.2 rad/sec respectively. The spawn location of the bot acts as one of the vertices of the square

3. square_closed_loop.py
This script makes the bot traverse a square defined by the co-ordinates of it's vertices. The linear & angular velocity and position to acheive the same by functions defined by the physics of the system which were referred from the GotoGoal.py file. irrespective of the spawn location, the bot first travels towards the first vetices defined by (5,5) unit. The bot sometimes sturgle in the vertices while orienting itself to the adjacent edge, hence perfect squares becomes an issue sometimes.


SEQUENCE FOR RUNNING PYTHON SCRIPT

In the catkin workspace, opne 3 seprate terminals in the given order with their commands as: 

TERMINAL 1: 
roscore

TERMINAL 2: 
rosrun turtlesim turtlesim_node

TERMINAL 3:
source devel/setup.bash
rosrun assignment2_turtlesim <file name>



