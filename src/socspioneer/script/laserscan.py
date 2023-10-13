#!/usr/bin/python3
# import turtle

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

pub = rospy.Publisher("distance", Float32, queue_size=100)


def calculate_colour(distance, max_distance):
    if distance <= 0.2:
        return "black"
    elif distance <= 0.7:
        return "red"
    elif distance <= 2:
        return "orange"
    elif distance <= 4:
        return "yellow"
    elif distance < max_distance:
        return "lime"
    else:
        return "green"


def callback(msg):
    print("TOMISPOG")

    # turtle.speed(0)
    # turtle.tracer(0, 0)  # This line disables update so that it's not as slow
    # turtle.pencolor("black")
    #
    # # This draws the line for each laser bean
    # for i in msg.ranges:
    #     turtle.pencolor(calculate_colour(i, msg.range_max))
    #
    #     turtle.forward(i * 75)
    #     turtle.backward(i * 75)
    #     turtle.left(180 / len(msg.ranges))

    lowest = min(msg.ranges)

    # This is to write TOMISPOG
    # turtle.penup()
    # turtle.setposition(-300, -50)
    # turtle.pendown()
    # turtle.pencolor(calculate_colour(lowest, msg.range_max))
    # turtle.write("TOMISPOG", font=("Verdana", 30, "normal"))
    # turtle.penup()
    # turtle.setposition(0, 0)
    # turtle.pendown()

    # Enabling update to draw the lines
    # turtle.update()
    # turtle.left(180)
    # turtle.clear()

    print(len(msg.ranges))
    print(lowest)

    right = msg.ranges[:100]
    right_avg = sum(right)/len(right)
    mid_right = msg.ranges[100:200]
    mid_right_avg = sum(mid_right)/len(mid_right)
    middle = msg.ranges[200:300]
    mid_avg = sum(middle)/len(middle)
    mid_left = msg.ranges[300:400]
    mid_left_avg = sum(mid_left)/len(mid_left)
    right = msg.ranges[400:]
    right_avg = sum(right)/len(right)

    # TODO try to implement the turn based on average distance?

    pub.publish(lowest)


def listener():
    rospy.init_node("LaserScan", anonymous=True)
    rospy.Subscriber("base_scan", LaserScan, callback)

    while not rospy.is_shutdown():
        # Do things here maybe
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
