#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from com760cw1_b01001406.msg import B01001406LeaderMessage


class FollowerController:

    def __init__(self):

        rospy.init_node("b01001406_follower_controller")

        self.state = "WAITING"
        self.turtle_name = rospy.get_param("~turtle_name")

        self.vel_pub = rospy.Publisher(
            f"/{self.turtle_name}/cmd_vel",
            Twist,
            queue_size=10
        )

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Leader instructions
        rospy.Subscriber(
            "/b01001406/leader_instruction",
            B01001406LeaderMessage,
            self.leader_instruction_callback
        )

        rospy.Timer(rospy.Duration(0.1), self.update)

        rospy.loginfo(f"{self.turtle_name} follower started (TF-based).")

    # ----------------------------------------------------------

    def leader_instruction_callback(self, msg):

        if msg.message == "FORM_UP":
            self.state = "FOLLOWING"

        elif msg.message == "RESET":
            self.state = "WAITING"
            self.stop()

        elif msg.message == "SPIN":
            self.state = "SPIN"
            self.spin_start_time = rospy.Time.now()    

    # ----------------------------------------------------------

    def update(self, event):

        if self.state != "FOLLOWING":
            return

        try:
            # Each follower has its own target frame
            target_frame = f"{self.turtle_name}_target"

            trans = self.tf_buffer.lookup_transform(
                self.turtle_name,
                target_frame,
                rospy.Time(0),
                rospy.Duration(0.1)
            )

            dx = trans.transform.translation.x
            dy = trans.transform.translation.y

            distance = math.sqrt(dx*dx + dy*dy)

            vel = Twist()

            if distance > 0.05:

                angle = math.atan2(dy, dx)

                vel.linear.x = min(1.5 * distance, 2.0)
                vel.angular.z = 4.0 * angle

                self.vel_pub.publish(vel)
                return

            # Orientation alignment
            rot = trans.transform.rotation
            yaw = tf_conversions.transformations.euler_from_quaternion(
                [rot.x, rot.y, rot.z, rot.w]
            )[2]

            if abs(yaw) > 0.05:
                vel.linear.x = 0.0
                vel.angular.z = 2.0 * yaw
                self.vel_pub.publish(vel)
                return

            self.stop()

        except:
            pass

    # ----------------------------------------------------------

    def stop(self):
        self.vel_pub.publish(Twist())


if __name__ == "__main__":
    try:
        FollowerController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
