#! /usr/bin/env python3

from Turtle import *

if __name__ == '__main__':
    num_other_turtles = 2
    rospy.init_node('Ejercicio3', anonymous = True)
    turtle_main = Turtle_controlled('turtle2')
    other_turtles = [Turtle('turtle1'),Turtle('turtle3')]
    if len(other_turtles) == 0:
        other_turtles = None
    time.sleep(3)
    points = [  [1,5.5],
                [5.5,10],
                [10,5.5],
                [5.5,1]]

    while True:
        for point in points:
            turtle_main.orientate(point[0], point[1])
            turtle_main.go_to_goal(point[0],point[1],other_turtles)
        turtle_main.orientate(desired_angle=1.57)