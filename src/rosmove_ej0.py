#! /usr/bin/env python3

from turtle import back
from Turtle import *

def square(turtle):
    wp = [[1,10],[10,10],[10,1],[1,1]]
    for point in wp:
        turtle.orientate(point[0],point[1])
        turtle.go_to_goal(point[0],point[1])

def circle(turtle):
    turtle.orientate(1,5.5)
    turtle.pure_pursuit(1,5.5)
    turtle.orientate(1,1)
    for i in range(16,361,16):
        turtle.pure_pursuit(5.5-np.cos(i*(np.pi/180))*(4.5),5.5-np.sin(i*(np.pi/180))*(4.5), velocity = 3)
    turtle.pure_pursuit(1,5.5)

if __name__ == '__main__':
    try:
        rospy.init_node('Ejercicio0', anonymous = True)
        turtle1 = Turtle_controlled('turtle1')
        time.sleep(3)
        square(turtle1)
        circle(turtle1)
    except rospy.ROSInterruptException:        
        pass
