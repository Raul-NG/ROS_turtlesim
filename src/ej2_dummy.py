#! /usr/bin/env python3

from Turtle import *
import random

if __name__ == '__main__':
    try:
        rospy.init_node('Dummy', anonymous = True)
        turtle2 = Turtle_controlled('turtle2')
        time.sleep(3)
        while True:
            turtle2.go_to_goal(5.5,1,velocity = random.randint(5,70)/10)
            turtle2.orientate(5.5,10)
            turtle2.go_to_goal(5.5,10,velocity = random.randint(5,70)/10)
            turtle2.orientate(5.5,1)

    except rospy.ROSInterruptException:        
        pass
