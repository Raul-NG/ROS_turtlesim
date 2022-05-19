#! /usr/bin/env python3

from tkinter.tix import Tree
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
        self.linear_velocity = pose_message.linear_velocity
        self.angular_velocity = pose_message.angular_velocity

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

    def __init__(self, name, max_speed=2.0):
        super().__init__(name)
        cmd_vel_topic = '/'+name+'/cmd_vel'
        self.velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)
        self.max_speed = 2.0

    def set_velocity(self, linear_velocity = 0.0, angular_velocity = 0.0):
        velocity_message = Twist()
        velocity_message.linear.x = linear_velocity
        velocity_message.angular.z = angular_velocity
        self.velocity_publisher.publish(velocity_message)

    def check_border_crash(self):
        if self.x > 10.3 and -math.pi/2 < self.theta and self.theta < math.pi/2:
            return True
        elif self.x < 0.7 and -math.pi/2 > self.theta and self.theta > math.pi/2:
            return True
        elif self.y > 10.3 and math.pi > self.theta and self.theta > 0:
            return True
        elif self.y < 0.7 and -math.pi < self.theta and self.theta < 0:
            return True
        return False

    def orientate(self, x_goal=None, y_goal=None, back = False, desired_angle = None):
        self.set_velocity()
        kap = 5.5
        kad = 1.5
        dt = 0.001
        desired_angle_goal = (math.atan2(y_goal-self.y, x_goal-self.x) - math.pi*back) if desired_angle is None else desired_angle
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

    def go_to_goal(self, x_goal, y_goal, other_turtles = None, velocity = None, go_back = False, threshold = 0.1, stop = True, look_ahead_distance = 2):
        kap = 8.0
        kad = 2.5
        klp = 2.0
        kld = 0.4
        dt = 0.001
        look_ahead_distance = 2.5
        num_distances = 3
        
        desired_angle_goal = math.atan2(y_goal-self.y, x_goal-self.x)
        dtheta = desired_angle_goal-(self.theta - math.pi*go_back)
        dtheta = math.atan2(math.sin(dtheta),math.cos(dtheta)) #Devuelve el menor angulo para girar
        distance = self.get_distance(x_goal, y_goal)
        while(distance > threshold):
            if self.check_border_crash():
                self.set_velocity()
                if 0.7 < x_goal and x_goal > 10.3 and 0.7 < y_goal and y_goal > 10.3:
                    self.go_to_goal(x_goal,y_goal, other_turtles)
                return

            time.sleep(dt)
            dtheta_ant = dtheta
            distance_ant = distance
            linear_speed = (1 - 2*go_back)*(klp * (distance) + kld * (distance - distance_ant)/dt if velocity is None else velocity)
            linear_speed = linear_speed if linear_speed < self.max_speed else self.max_speed

            if other_turtles is not None:
                for turtle in other_turtles:
                    pos_turtle = self.tp_global_to_turtle(turtle.x,turtle.y)
                    if self.get_distance(turtle=turtle) < max(look_ahead_distance, turtle.linear_velocity) and pos_turtle[0] > 0 and abs(pos_turtle[1]) < 1:
                        theta_ant = self.theta
                        m = (y_goal - self.y)/(x_goal - self.x)
                        b = self.y - m*self.x
                        point_x = pos_turtle[0] - max(1,turtle.linear_velocity)
                        if pos_turtle[1] < 0 and math.cos(turtle.theta - self.theta) >= 0:# and abs(turtle.tp_global_to_turtle(self.x, self.y)[1]) < 1.5
                            point_y = pos_turtle[1] + 1 #izquierda
                        else: # pos_turtle[1] >= 0:
                            point_y = pos_turtle[1] - 1 #derecha

                        point = self.tp_turtle_to_global(point_x, point_y)

                        self.go_to_goal(point[0], point[1], velocity=self.max_speed)
                        dtheta = theta_ant-(self.theta - math.pi*go_back)
                        dtheta=math.atan2(math.sin(dtheta),math.cos(dtheta))
                        while abs(dtheta) > 0.001 and pos_turtle[0] >= -max(1.5, turtle.linear_velocity): # and turtle.tp_global_to_turtle(x_goal,y_goal)[0] >= -1.5:
                            time.sleep(dt)
                            pos_turtle = self.tp_global_to_turtle(turtle.x,turtle.y)
                            dtheta_ant = dtheta
                            dtheta = theta_ant - (self.theta - math.pi*go_back)
                            dtheta=math.atan2(math.sin(dtheta),math.cos(dtheta))
                            angular_speed = kap * (dtheta) + kad * (dtheta - dtheta_ant)/dt
                            self.set_velocity(self.max_speed, angular_speed)
                            if self.check_border_crash():
                                self.set_velocity()
                                if 0.7 < x_goal and x_goal > 10.3 and 0.7 < y_goal and y_goal > 10.3:
                                    self.go_to_goal(x_goal,y_goal, other_turtles)
                                break

                        while pos_turtle[0] >= -1.5 and turtle.tp_global_to_turtle(x_goal,y_goal)[0] >= -1.5:
                            self.set_velocity(linear_speed)
                            time.sleep(dt)
                            pos_turtle = self.tp_global_to_turtle(turtle.x,turtle.y)
                            if self.check_border_crash():
                                self.set_velocity()
                                if 0.7 < x_goal and x_goal > 10.3 and 0.7 < y_goal and y_goal > 10.3:
                                    self.go_to_goal(x_goal,y_goal, other_turtles)
                                break
                        goal = self.tp_global_to_turtle(x_goal, y_goal)
                        pos = [self.x, self.y] if goal[0] <= 0 else self.tp_turtle_to_global(1,0)
                        x_line = (pos[1] + pos[0]/m - b)/(m + 1/m)
                        y_line = m*x_line+b
                        # self.go_to_goal((3*x_line + self.x)/4,(3*y_line+self.y)/4,threshold=0.1,velocity=self.max_speed,stop=False)

                        # self.go_to_goal_dodge(x_line,y_line,other_turtles,velocity=linear_speed,threshold=0.1,look_ahead_distance=0.5)
                        if turtle.get_distance(x_goal, y_goal) < 1.5 and self.get_distance(x_goal, y_goal) > 1.5:
                            self.orientate(x_goal,y_goal)
                        elif self.tp_global_to_turtle(x_goal,y_goal)[0] > self.tp_global_to_turtle(x_line,y_line)[0] or turtle.get_distance(x_goal, y_goal) < 1.5:
                            self.go_to_goal(x_line,y_line,threshold=0.1,velocity=self.max_speed,stop=False)
                        
                        while turtle.get_distance(x_goal, y_goal) < 1.5 and self.get_distance(x_goal, y_goal) > 1.5:
                            self.orientate(x_goal,y_goal)
                            time.sleep(dt)

            desired_angle_goal = math.atan2(y_goal-self.y, x_goal-self.x)
            dtheta = desired_angle_goal - (self.theta - math.pi*go_back)
            dtheta=math.atan2(math.sin(dtheta),math.cos(dtheta)) # Devuelve el menor angulo para girar
            distance = self.get_distance(x_goal, y_goal)
            
            angular_speed = kap * (dtheta) + kad * (dtheta - dtheta_ant)/dt
            # if abs(angular_speed) > 13:
            #     self.orientate(x_goal, y_goal)

            self.set_velocity(linear_speed, angular_speed)
        if stop:
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