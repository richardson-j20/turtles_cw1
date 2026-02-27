#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose


class TurtleTFBroadcaster:

    def __init__(self):

        rospy.init_node("turtle_tf_broadcaster")

        # Get turtle name from launch file parameter
        self.turtle_name = rospy.get_param("~turtle")

        # Create one broadcaster instance (not per callback)
        self.br = tf2_ros.TransformBroadcaster()

        # Subscribe to turtlesim pose
        rospy.Subscriber(
            f"/{self.turtle_name}/pose",
            Pose,
            self.pose_callback
        )

        rospy.loginfo(f"TF broadcaster started for {self.turtle_name}")

        rospy.spin()

    # ----------------------------------------------------------

    def pose_callback(self, msg):

        t = TransformStamped()

        # Time stamp
        t.header.stamp = rospy.Time.now()

        # Parent frame
        t.header.frame_id = "world"

        # Child frame
        t.child_frame_id = self.turtle_name

        # Position
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # Orientation (convert yaw to quaternion)
        q = tf_conversions.transformations.quaternion_from_euler(
            0,
            0,
            msg.theta
        )

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast transform
        self.br.sendTransform(t)

if __name__ == "__main__":
    try:
        TurtleTFBroadcaster()
    except rospy.ROSInterruptException:
        pass
