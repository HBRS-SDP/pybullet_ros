#!/usr/bin/env python3


## A sample code to republish the /joint_states with the topic name joint_state_publsher if needed

import rospy
from sensor_msgs.msg import JointState

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "publish joint_state %s", data)
    rospy.loginfo("Publishing")



def talker():
    pub = rospy.Publisher('joint_state_publisher', JointState, queue_size=1)
    sub=rospy.Subscriber("/joint_states", JointState, callback)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 50hz
    while not rospy.is_shutdown():
        
        pub.publish(sub)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass