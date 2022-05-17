#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
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

    def tp_global_to_turtle(self, x_global, y_global):
        R_t = np.transpose(np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]]))
        d = np.array([[self.x],[self.y]])
        return np.transpose(R_t.dot(np.array([[x_global],[y_global]]) - d))[0].tolist()
    
    def tp_turtle_to_global(self, x_turtle, y_turtle):
        R = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
        d = np.array([[self.x],[self.y]])
        return np.transpose(R.dot(np.array([[x_turtle],[y_turtle]])) + d)[0].tolist()

    def get_distance(self, x=None, y=None, turtle=None):
        if turtle is None:
            return math.sqrt((self.x - x)**2 + (self.y - y)**2)
        else:
            return math.sqrt((self.x - turtle.x)**2 + (self.y - turtle.y)**2)
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
        kap = 5.5
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

    def go_to_goal(self, x_goal, y_goal, velocity = None, go_back = False, threshold = 0.1):
        kap = 8.0
        kad = 2.5
        klp = 2.0
        kld = 0.4
        dt = 0.001
        look_ahead_distance = 2.5
        
        desired_angle_goal = math.atan2(y_goal-self.y, x_goal-self.x)
        dtheta = desired_angle_goal-(self.theta - math.pi*go_back)
        dtheta=math.atan2(math.sin(dtheta),math.cos(dtheta)) #Devuelve el menor angulo para girar
        distance = self.get_distance(x_goal, y_goal)

        while(distance > threshold):
            dtheta_ant = dtheta
            distance_ant = distance
            linear_speed = (1 - 2*go_back)*(klp * (distance) + kld * (distance - distance_ant)/dt if velocity is None else velocity)
            linear_speed = linear_speed if linear_speed < 2 else 2

            time.sleep(dt)
            desired_angle_goal = math.atan2(y_goal-self.y, x_goal-self.x)
            dtheta = desired_angle_goal - (self.theta - math.pi*go_back)
            dtheta=math.atan2(math.sin(dtheta),math.cos(dtheta)) # Devuelve el menor angulo para girar
            distance = self.get_distance(x_goal, y_goal)
            
            angular_speed = kap * (dtheta) + kad * (dtheta - dtheta_ant)/dt
            self.set_velocity(linear_speed, angular_speed)
        self.set_velocity()

    def pure_pursuit(self, x_goal, y_goal, velocity = None, threshold = 0.05, go_back = False):
        distance = self.get_distance(x_goal, y_goal)
        while(distance > threshold):
            distance = self.get_distance(x_goal, y_goal)
            linear_speed = (1 - 2*go_back)*(distance if velocity is None else velocity)
            goal_transformed = self.tp_global_to_turtle(x_goal, y_goal)
            try:
                angular_speed = linear_speed * 2 * goal_transformed[1] / distance**2
            except ZeroDivisionError:
                angular_speed = 0
            self.set_velocity(linear_speed, angular_speed)
        self.set_velocity()

    def go_to_goal_dodge(self, x_goal, y_goal, other_turtles, velocity = None, go_back = False, threshold_goal = 0.1):
        kap = 6.5
        kad = 2.0
        klp = 1.5
        kld = 0.3
        dt = 0.001
        look_ahead_distance = 2.5
        
        desired_angle_goal = math.atan2(y_goal-self.y, x_goal-self.x)
        dtheta = desired_angle_goal-(self.theta - math.pi*go_back)
        dtheta = math.atan2(math.sin(dtheta),math.cos(dtheta))
        distance = self.get_distance(x_goal, y_goal)

        while(distance > threshold_goal):
            dtheta_ant = dtheta
            distance_ant = distance
            linear_speed = (1 - 2*go_back)*(klp * (distance) + kld * (distance - distance_ant)/dt if velocity is None else velocity)
            linear_speed if linear_speed < 2 else 2
            for turtle in other_turtles:
                pos_turtle = self.tp_global_to_turtle(turtle.x,turtle.y)
                if self.get_distance(turtle=turtle) < look_ahead_distance + turtle.linear_velocity and pos_turtle[0] > 0: #linear_speed + 1.5 + turtle.linear_velocity:
                    points = [self.tp_turtle_to_global(pos_turtle[0] - 1.2, pos_turtle[1] - 1.2),
                            self.tp_turtle_to_global(pos_turtle[0], pos_turtle[1] - 1.2),
                            self.tp_turtle_to_global(pos_turtle[0] + 1.2, 0)]
                    for point in points:
                        self.go_to_goal(point[0],point[1],velocity=linear_speed,threshold=0.1)
                    self.orientate(x_goal, y_goal)

            time.sleep(dt)
            desired_angle_goal = math.atan2(y_goal-self.y, x_goal-self.x)
            dtheta = desired_angle_goal - (self.theta - math.pi*go_back)
            dtheta=math.atan2(math.sin(dtheta),math.cos(dtheta))
            distance = self.get_distance(x_goal, y_goal)

            angular_speed = kap * (dtheta) + kad * (dtheta - dtheta_ant)/dt
            
            self.set_velocity(linear_speed, angular_speed)
        self.set_velocity()