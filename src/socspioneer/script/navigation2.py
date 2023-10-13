import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)


def callback(msg):
    lowest = [msg.data[0], msg.data[1]]
    highest = [msg.data[2], msg.data[3]]
    averages = [msg.data[4], msg.data[5], msg.data[6], msg.data[7], msg.data[8]]

    # position = msg.data[1]

    base_data = Twist()

    print(lowest)
    print(highest)
    print(averages)

    if averages[4] < 2:
        base_data.linear.x = 1
    elif averages[4] > 1:
        base_data.angular.z = 1
        base_data.linear.x = 0.3
        # print("too far from left wall")
    elif averages[4] < 0.8:
        # print("too close to right wall")
        base_data.angular.z = -1
        base_data.linear.x = 0
    elif averages[2] > 0.5:
        # print("going forward")
        base_data.linear.x = 0.5
    if averages[2] <= 1:
        # print("avoiding wall in front of me")
        base_data.angular.z = -4
        base_data.linear.x = 0
    else:
        base_data.linear.x = 0.1
        # print("wtf")
    #    else:
    #        base_data.angular.z = -4
    #    base_data.linear.x = 0
    pub.publish(base_data)
    # else:
    #    base_data.linear.x = 1
    #    pub.publish(base_data)
    # rospy.sleep(rospy.Duration(1, 0))

    print("=========")


def listener():
    rospy.init_node("Navigation", anonymous=True)
    rospy.Subscriber('proximity_sensor', Float64MultiArray, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
