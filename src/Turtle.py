#! /usr/bin/env python3

from turtle import distance, pos
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
import numpy as np

class Turtle:
    def __init__(self, name):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.linear_velocity = 0
        self.angular_velocity = 0

        position_topic = '/'+name+"/pose"
        self.pose_subscriber = rospy.Subscriber(position_topic, Pose, self.poseCallback)

    def poseCallback(self, pose_message):
        self.x = pose_message.x
        self.y = pose_message.y
        self.theta = pose_message.theta
        self.vel = pose_message.linear_velocity
        self.omega = pose_message.angular_velocity

    def transform_point(self, x_global, y_global):
        R_t = np.transpose(np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]]))
        d = np.array([[self.x],[self.y]])
        return R_t.dot(np.array([[x_global],[y_global]]) - d).tolist()

class Turtle_controlled(Turtle):

    def __init__(self, name):
        super().__init__(name)
        cmd_vel_topic = '/'+name+'/cmd_vel'
        self.velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

    def set_velocity(self, linear_velocity = 0.0, angular_velocity = 0.0):
        velocity_message = Twist()
        velocity_message.linear.x = linear_velocity
        velocity_message.angular.z = angular_velocity
        self.velocity_publisher.publish(velocity_message)

    def orientate(self, x_goal, y_goal, back = False):
        self.set_velocity()
        kap = 5.0
        kad = 1.5
        dt = 0.001
        desired_angle_goal = math.atan2(y_goal-self.y, x_goal-self.x) - math.pi*back
        dtheta = desired_angle_goal-self.theta
        dtheta=math.atan2(math.sin(dtheta),math.cos(dtheta)) #Devuelve el menor angulo para girar
        while(abs(dtheta) > 0.001):
            time.sleep(dt)
            dtheta_ant = dtheta
            dtheta = desired_angle_goal-self.theta
            dtheta = math.atan2(math.sin(dtheta),math.cos(dtheta))

            angular_speed =  kap*dtheta + kad*(dtheta - dtheta_ant)/dt
            self.set_velocity(angular_velocity=angular_speed)
        self.set_velocity()

    def go_to_goal(self, x_goal, y_goal, velocity = None, go_back = False, threshold = 0.01): # ¿theta_goal = None, threshold_angle  = 0.001?
        kap = 6.5
        kad = 2.0
        klp = 1.5
        kld = 0.3
        dt = 0.001
        
        desired_angle_goal = math.atan2(y_goal-self.y, x_goal-self.x)
        dtheta = desired_angle_goal-(self.theta - math.pi*go_back)
        dtheta=math.atan2(math.sin(dtheta),math.cos(dtheta)) #Devuelve el menor angulo para girar
        distance = math.sqrt((self.x - x_goal)**2 + (self.y - y_goal)**2)

        while(distance > threshold):
            time.sleep(dt)
            dtheta_ant = dtheta
            distance_ant = distance

            desired_angle_goal = math.atan2(y_goal-self.y, x_goal-self.x)
            dtheta = desired_angle_goal - (self.theta - math.pi*go_back)
            dtheta=math.atan2(math.sin(dtheta),math.cos(dtheta)) #Devuelve el menor angulo para girar
            distance = math.sqrt((self.x - x_goal)**2 + (self.y - y_goal)**2)
            
            angular_speed = kap * (dtheta) + kad * (dtheta - dtheta_ant)/dt
            linear_speed = (1 - 2*go_back)*(klp * (distance) + kad * (distance - distance_ant)/dt if velocity is None else velocity)
            self.set_velocity(linear_speed, angular_speed)
        self.set_velocity()

    def pure_pursuit(self, xgoal, ygoal, velocity = None, threshold = 0.05, go_back = False):
        distance = math.sqrt(((xgoal-self.x)**2)+((ygoal-self.y)**2))
        while(distance > threshold):
            distance = math.sqrt(((xgoal-self.x)**2)+((ygoal-self.y)**2))
            linear_speed = (1 - 2*go_back)*(distance if velocity is None else velocity)
            goal_transformed = self.transform_point(xgoal, ygoal)
            try:
                angular_speed = linear_speed * 2 * goal_transformed[1][0] / distance**2
            except ZeroDivisionError:
                angular_speed = 0
            self.set_velocity(linear_speed, angular_speed)
        self.set_velocity()