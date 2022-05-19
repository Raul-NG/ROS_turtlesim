#! /usr/bin/env python3

from Turtle import *
import random

if __name__ == '__main__':
    try:
        rospy.init_node('Dummy2', anonymous = True)
        turtle3 = Turtle_controlled('turtle3')
        other_turtles = [Turtle('turtle1'),Turtle('turtle3')]
        time.sleep(3)
        points = [[5.5,10],
              [1,5.5],
              [10,5.5],
              [1,1],
              [10,1]]

        while True:
            for point in points:
                turtle3.orientate(point[0], point[1])
                turtle3.go_to_goal(point[0],point[1],other_turtles)
            turtle3.orientate(desired_angle=1.57)

    except rospy.ROSInterruptException:        
        pass
