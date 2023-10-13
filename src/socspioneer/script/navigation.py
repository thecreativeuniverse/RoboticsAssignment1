#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)


def callback(data):
    print(data)
    base_data = Twist()
    if data.data < 0.5:
        base_data.angular.z = 8
        base_data.linear.x = 0
        pub.publish(base_data)
        print("---TURN---")
    else:
        base_data.linear.x = 0.1
        pub.publish(base_data)
    # rospy.sleep(rospy.Duration(1, 0))


def listener():
    rospy.init_node("Navigation", anonymous=True)
    rospy.Subscriber("distance", Float32MultiArray, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
