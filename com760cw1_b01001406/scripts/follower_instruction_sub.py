#!/usr/bin/env python3
import rospy
from com760cw1_b01001406.msg import B01001406LeaderMessage

def callback(data):
    rospy.loginfo("[%s] instructionID=%d message='%s'",
                  rospy.get_name(), data.instructionID, data.message)

def follower_listener():
    rospy.init_node("b01001406_follower_listener", anonymous=True)
    rospy.Subscriber("/b01001406/leader_instruction", B01001406LeaderMessage, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        follower_listener()
    except rospy.ROSInterruptException:
        pass
