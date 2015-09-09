#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from random import randrange

def talker():
    pub = rospy.Publisher('ir1', Range, queue_size=10)
    rospy.init_node('range_publisher', anonymous=True)
    r = rospy.Rate(10) # 10hz
    msg = Range()
    while not rospy.is_shutdown():
        msg = Range()
        msg.range = randrange(0,1000)/900.
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass