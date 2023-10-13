import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
# import turtle

pub = rospy.Publisher('/proximity_sensor', Float64MultiArray, queue_size=10)


def calculateColour(distance, max_distance):
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
    elif distance == max_distance:
        return "green"


def callback(msg):
    # print("TOMISPOG")

    # turtle.speed(0)
    # turtle.tracer(0, 0)
    # turtle.pencolor("black")
    #
    # for i in msg.ranges:
    #     turtle.pencolor(calculateColour(i, msg.range_max))
    #
    #     turtle.forward(i * 75)
    #     turtle.backward(i * 75)
    #     turtle.left(180 / len(msg.ranges))
    #
    # turtle.penup()
    # turtle.setposition(-300, -50)
    # turtle.pendown()

    lowest = min(msg.ranges)
    lowest_location = msg.ranges.index(lowest)

    highest = max(msg.ranges)
    highest_location = msg.ranges.index(highest)

    right = msg.ranges[:100]
    right_avg = sum(right) / len(right)

    middle_right = msg.ranges[100:200]
    middle_right_avg = sum(middle_right) / len(middle_right)

    middle = msg.ranges[200:300]
    middle_avg = sum(middle) / len(middle)

    middle_left = msg.ranges[300:400]
    middle_left_avg = sum(middle_left) / len(middle_left)

    left = msg.ranges[400:]
    left_avg = sum(left) / len(left)

    # turtle.pencolor(calculateColour(lowest, msg.range_max))
    #
    # turtle.write("TOMISPOG", font=("Verdana", 30, "normal"))
    #
    # turtle.penup()
    # turtle.setposition(0, 0)
    # turtle.pendown()
    #
    # turtle.update()
    # turtle.left(180)
    # turtle.clear()

    stuff = Float64MultiArray()
    stuff.data = [lowest, lowest_location, highest, highest_location, right_avg, middle_right_avg, middle_avg, middle_left_avg,
                  left_avg]
    pub.publish(stuff)


def listener():
    rospy.init_node('proximity_sensor', anonymous=True)
    rospy.Subscriber('base_scan', LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
