#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import numpy as np

class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot1',anonymous=True)
        
        #Define the publisher
        self.pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        
        #Define the subscriber 
        self.pose_subscribe = rospy.Subscriber('/turtle1/pose',Pose,self.pose_update)
        
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        
        #Defines the pose of the bot 
    def pose_update(self,data):
        self.pose = data
        self.pose.x = round(self.pose.x,4)
        self.pose.y = round(self.pose.y,4)
        
        #Calculates the euclidean distance between the goal pose and the current pose
    def euclidean_distance(self,goal_pose):
        return sqrt(pow((goal_pose.x-self.pose.x),2)+pow((goal_pose.y-self.pose.y),2))
    
        #calculates the linear speed based on the distance to goal
    def linear_vel(self,goal_pose,const=1.5):
        return const*self.euclidean_distance(goal_pose)
    
         #calculates the steering angle based on the vector it is supposed to be headed
    def steering_angle(self,goal_pose):
        return atan2((goal_pose.y-self.pose.y),(goal_pose.x-self.pose.x))
    
         #returns the required steering angle based on the difference between the steering angle and the bot orientation
    def angular_vel(self,goal_pose,const=6):
        return const*(self.steering_angle(goal_pose)-self.pose.theta)
    
        #Defines the movement of the bot 
    def move_square(self):
        #defining the vertices of the square
        square_x = np.array([5,8,8,5,5],dtype=float)    
        square_y = np.array([5,5,8,8,5],dtype=float)    
        dist_tol = 0.1           #distance tolerance 
        
        goal_pose = Pose()
        vel_msgs = Twist()
        
        #first goal pose is the first vertex 
        goal_pose.x = (square_x[0])
        goal_pose.y = (square_y[0])
        
        while self.euclidean_distance(goal_pose) > dist_tol:
            
            #setting the velocities to reach the first vertex
            vel_msgs.linear.x = self.linear_vel(goal_pose)
            vel_msgs.linear.y = 0
            vel_msgs.linear.z = 0
            vel_msgs.angular.x = 0
            vel_msgs.angular.y = 0
            vel_msgs.angular.z = self.angular_vel(goal_pose)
            self.pub.publish(vel_msgs)
            self.rate.sleep()
        
        vel_msgs.linear.x = 0
        vel_msgs.angular.z = 0
        self.pub.publish(vel_msgs)
        
        
        #the next step is to align it with the next edge  -- 
        #and then travel along it

        i = 1   #since it has already reached the first vertex
        
        while i < 5:
            goal_pose.x = (square_x[i])
            goal_pose.y = (square_y[i])
            diff = abs(self.steering_angle(goal_pose)-self.pose.theta) 
            angle_tol = 1   #angle tolerance 
            
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()
            
            #first rotate the bot to align with the next edge
            while diff >= angle_tol:
                if (t1-t0 <= 4):
                    #setting the velocities to align with the next edge
                    vel_msgs.linear.x = 0
                    vel_msgs.linear.y = 0
                    vel_msgs.linear.z = 0
                    vel_msgs.angular.x = 0
                    vel_msgs.angular.y = 0
                    vel_msgs.angular.z = self.angular_vel(goal_pose)
                    self.pub.publish(vel_msgs)
                    t1 = rospy.Time.now().to_sec()
                else:
                    break
                        
            
            #angular speed is zero once the vertex is aligned
            vel_msgs.angular.z = 0
            self.pub.publish(vel_msgs)
            
            current_distance = 0
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()
            
            #moving the bot to the next vertex
            while current_distance <= 3:
                if (t1-t0 <= 4):
                    #setting the velocities to reach the first vertex
                    vel_msgs.linear.x = self.linear_vel(goal_pose)
                    vel_msgs.linear.y = 0
                    vel_msgs.linear.z = 0
                    vel_msgs.angular.x = 0
                    vel_msgs.angular.y = 0
                    vel_msgs.angular.z = 0
                    self.pub.publish(vel_msgs)
                    t1 = rospy.Time.now().to_sec()
        		    #calculating the distance travelled
                    current_distance = (vel_msgs.linear.x)*(t1-t0)
                else:
                    break
            
            #set the linear speed to zero once the vertex is reached
            vel_msgs.linear.x = 0
            self.pub.publish(vel_msgs)
            
            i = i+1
                

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move_square()
    except rospy.ROSInterruptException:pass
