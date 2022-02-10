#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)     # (topic_name, msg_type, queue_size)
    rospy.init_node('talker', anonymous=True)                   # (node_name, arg)
                                                                # anonymous allows multiple duplicate nodes run at the same time
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

# if the executed file is this, then run the main code, otherwise the classes above will be used
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass