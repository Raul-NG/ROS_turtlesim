#! /usr/bin/env python3

from Turtle import *

if __name__ == '__main__':
    rospy.init_node('Ejercicio1', anonymous = True)
    turtle_main = Turtle_controlled('turtle1')
    other_turtles = [Turtle('turtle'+str(i)) for i in range(2,3)]
    time.sleep(3)

    while True:
        turtle_main.go_to_goal(5.5,9,other_turtles)
        turtle_main.orientate(5.5,2)
        turtle_main.go_to_goal(5.5,2,other_turtles) 
        turtle_main.orientate(5.5,9)