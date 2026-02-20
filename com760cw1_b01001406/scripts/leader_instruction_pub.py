#!/usr/bin/env python3
import rospy
from com760cw1_b01001406.msg import B01001406LeaderMessage

def leader_publisher():
    pub = rospy.Publisher("/b01001406/leader_instruction", B01001406LeaderMessage, queue_size=10)
    rospy.init_node("b01001406_leader_publisher", anonymous=True)

    # publish once
    msg = B01001406LeaderMessage()
    msg.instructionID = 0
    msg.message = "Move to formation: FollowerA 1m left, FollowerB 1m right"

    rate = rospy.Rate(2)
    for _ in range(5):
        pub.publish(msg)
        rospy.loginfo(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        leader_publisher()
    except rospy.ROSInterruptException:
        pass
