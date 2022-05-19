#! /usr/bin/env python3

from Turtle import *
import random

if __name__ == '__main__':
    try:
        rospy.init_node('Dummy1', anonymous = True)
        turtle1 = Turtle_controlled('turtle1')
        other_turtles = [Turtle('turtle1'),Turtle('turtle3')]
        time.sleep(3)

        points = [[10,5.5],
              [1,5.5],
              [5.5,10],
              [10,1],
              [1,1]]

        while True:
            for point in points:
                turtle1.orientate(point[0], point[1])
                turtle1.go_to_goal(point[0],point[1],other_turtles)
            turtle1.orientate(desired_angle=1.57)
            # turtle2.go_to_goal(5.5,2,velocity = 5)
            # while True:
            #     turtle2.orientate(5.5,9)
            #     turtle2.go_to_goal(5.5,9,velocity = random.randint(5,70)/10)
            #     turtle2.orientate(5.5,2)
            #     turtle2.go_to_goal(5.5,2,velocity = random.randint(5,70)/10)

    except rospy.ROSInterruptException:        
        pass
