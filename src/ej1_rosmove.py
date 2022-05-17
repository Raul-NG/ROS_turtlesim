#! /usr/bin/env python3

from Turtle import *

if __name__ == '__main__':
    rospy.init_node('Ejercicio1', anonymous = True)
    turtle_main = Turtle_controlled('turtle1')
    other_turtles = [Turtle('turtle'+str(i)) for i in range(2,3)]
    time.sleep(3)

    while turtle_main.y < 10:
        if (math.sqrt((turtle_main.x - other_turtles[0].x)**2 + (turtle_main.y - other_turtles[0].y)**2)) >= 1.5 + other_turtles[0].linear_velocity:
            turtle_main.set_velocity(2)
            continue

        while (math.sqrt((turtle_main.x - other_turtles[0].x)**2 + (turtle_main.y - other_turtles[0].y)**2)) < 1.5:
            if turtle_main.y >= other_turtles[0].y:
                turtle_main.set_velocity(5)
            else:
                turtle_main.set_velocity(-1)
        
        while turtle_main.y < other_turtles[0].y and abs(other_turtles[0].x - turtle_main.x) < 1:
            turtle_main.set_velocity()
    for i in range(5):
        turtle_main.set_velocity()

    # threshold = (turtle1.vel*turtle2.vel) + 0.5 #if  (turtle1.vel*turtle2.vel) == 0 else 1
    # for i in range(0,10):
    #     while (math.sqrt((turtle_main.x - other_turtles[0].x)**2 + (turtle_main.y - other_turtles[0].y)**2)) < 1:
    #         # turtle1.go_to_goal_back(turtle1.x,turtle1.y-1)
    #         turtle_main.go_to_goal(turtle_main.x,turtle_main.y-1,go_back=True)
    #         #pass
    #     turtle_main.go_to_goal(5.5, i+1, 3)
