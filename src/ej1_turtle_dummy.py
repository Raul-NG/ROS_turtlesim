#! /usr/bin/env python3

from Turtle import *
import random

if __name__ == '__main__':
    try:
        rospy.init_node('Dummy', anonymous = True)
        turtle2 = Turtle_controlled('turtle2')
        time.sleep(3)
        while True:
            #Velocidad en 1.5, regresa; velocidad en 1.29, acelera
            turtle2.go_to_goal(6,5.5,velocity = 1.5)#1.29)#random.randint(5,70)/10)
            time.sleep(3)
            turtle2.go_to_goal(10,5.5,velocity = random.randint(5,70)/10)
            turtle2.orientate(1,5.5)
            turtle2.go_to_goal(1,5.5)
            turtle2.orientate(3,5.5)

    except rospy.ROSInterruptException:        
        pass
