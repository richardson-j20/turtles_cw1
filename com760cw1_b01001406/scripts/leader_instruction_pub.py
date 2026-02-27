#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import math
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from com760cw1_b01001406.msg import B01001406LeaderMessage
from turtlesim.srv import TeleportAbsolute, SetPen
from com760cw1_b01001406.srv import B01001406GoToTarget
from std_srvs.srv import Empty

class LeaderController:

    def __init__(self):

        rospy.init_node("b01001406_leader_controller")

        self.state = "FORMING"
        self.pose = Pose()
        self.target_angle = None

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.formation_br = tf2_ros.TransformBroadcaster()

        # ---------------- Publishers ----------------
        self.pub = rospy.Publisher(
            "/b01001406/leader_instruction",
            B01001406LeaderMessage,
            queue_size=10
        )

        self.vel_pub = rospy.Publisher(
            "/b01001406Leader/cmd_vel",
            Twist,
            queue_size=10
        )

        rospy.Subscriber(
            "/b01001406Leader/pose",
            Pose,
            self.pose_callback
        )

        # ---------------- Services ----------------
        self.teleport_A = rospy.ServiceProxy(
            "/b01001406FollowerA/teleport_absolute",
            TeleportAbsolute
        )

        self.teleport_B = rospy.ServiceProxy(
            "/b01001406FollowerB/teleport_absolute",
            TeleportAbsolute
        )

        self.teleport_leader = rospy.ServiceProxy(
            "/b01001406Leader/teleport_absolute",
            TeleportAbsolute
        )

        self.go_to_goal_client = rospy.ServiceProxy(
            "go_to_target_b01001406Leader",
            B01001406GoToTarget
        )

        self.pen_leader = rospy.ServiceProxy(
            "/b01001406Leader/set_pen",
            SetPen
        )

        self.pen_A = rospy.ServiceProxy(
            "/b01001406FollowerA/set_pen",
            SetPen
        )

        self.pen_B = rospy.ServiceProxy(
            "/b01001406FollowerB/set_pen",
            SetPen
        )

        self.clear = rospy.ServiceProxy("/clear", Empty)

        rospy.Timer(rospy.Duration(0.1), self.run)

        rospy.loginfo("Leader controller started.")

    # ----------------------------------------------------------

    def pose_callback(self, msg):
        self.pose = msg

    # ----------------------------------------------------------
    # TF Formation Frame Broadcaster
    # ----------------------------------------------------------

    def broadcast_formation_frames(self):

        for name, offset_y in [
            ("b01001406FollowerA_target", 1.0),
            ("b01001406FollowerB_target", -1.0)
        ]:

            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "b01001406Leader"
            t.child_frame_id = name

            t.transform.translation.x = -1.0
            t.transform.translation.y = offset_y
            t.transform.translation.z = 0.0

            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.formation_br.sendTransform(t)

    # ----------------------------------------------------------

    def run(self, event):

        # Always broadcast formation frames
        self.broadcast_formation_frames()

        if self.state == "FORMING":
            self.publish_instruction()
            if self.formation_complete():
                rospy.loginfo("Formation complete.")
                self.target_angle = random.uniform(0, 2 * math.pi)
                self.state = "TURNING"

        elif self.state == "TURNING":
            if self.rotate_to_angle(self.target_angle):
                rospy.loginfo("Leader turned.")
                self.state = "MOVING"

        elif self.state == "MOVING":
            self.move_forward()
            self.check_boundary()

        elif self.state == "RESETTING":
            self.reset_system()

        elif self.state == "SPIN":
            self.spin_leader()    

    # ----------------------------------------------------------

    def publish_instruction(self):

        msg = B01001406LeaderMessage()
        msg.instructionID = rospy.Time.now().secs
        msg.message = "FORM_UP"
        self.pub.publish(msg)

    # ----------------------------------------------------------

    def formation_complete(self):

        try:
            transA = self.tf_buffer.lookup_transform(
                "b01001406Leader",
                "b01001406FollowerA",
                rospy.Time(0),
                rospy.Duration(0.1)
            )

            transB = self.tf_buffer.lookup_transform(
                "b01001406Leader",
                "b01001406FollowerB",
                rospy.Time(0),
                rospy.Duration(0.1)
            )

            errorA = math.sqrt(
                (transA.transform.translation.x + 1.0)**2 +
                (transA.transform.translation.y - 1.0)**2
            )

            errorB = math.sqrt(
                (transB.transform.translation.x + 1.0)**2 +
                (transB.transform.translation.y + 1.0)**2
            )

            rotA = transA.transform.rotation
            rotB = transB.transform.rotation

            yawA = tf_conversions.transformations.euler_from_quaternion(
                [rotA.x, rotA.y, rotA.z, rotA.w]
            )[2]

            yawB = tf_conversions.transformations.euler_from_quaternion(
                [rotB.x, rotB.y, rotB.z, rotB.w]
            )[2]

            orientation_ok = abs(yawA) < 0.1 and abs(yawB) < 0.1

            return errorA < 0.1 and errorB < 0.1 and orientation_ok

        except:
            return False

    # ----------------------------------------------------------

    def rotate_to_angle(self, target):

        error = target - self.pose.theta
        error = math.atan2(math.sin(error), math.cos(error))

        if abs(error) < 0.05:
            self.vel_pub.publish(Twist())
            return True

        vel = Twist()
        vel.angular.z = 2.5 * error
        self.vel_pub.publish(vel)
        return False

    # ----------------------------------------------------------

    def move_forward(self):

        vel = Twist()
        vel.linear.x = 2.0
        self.vel_pub.publish(vel)

    # ----------------------------------------------------------

    def check_boundary(self):

        margin = 0.8  # safety buffer

        if (
            self.pose.x <= margin or
            self.pose.x >= 11.0 - margin or
            self.pose.y <= margin or
            self.pose.y >= 11.0 - margin
        ):
            rospy.loginfo("Boundary reached safely. Resetting.")
            self.vel_pub.publish(Twist())
            self.state = "RESETTING"

    # ----------------------------------------------------------

    def rotate_to_zero(self):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            error = 0.0 - self.pose.theta
            error = math.atan2(math.sin(error), math.cos(error))

            if abs(error) < 0.02:
                self.vel_pub.publish(Twist())
                break

            vel = Twist()
            vel.angular.z = 3.0 * error
            self.vel_pub.publish(vel)
            rate.sleep()

    # ----------------------------------------------------------

    def reset_system(self):

        try:

            # Notify followers
            msg = B01001406LeaderMessage()
            msg.instructionID = rospy.Time.now().secs
            msg.message = "RESET"
            self.pub.publish(msg)

            rospy.sleep(0.5)

            # White pens
            self.pen_leader(255, 255, 255, 2, 0)
            self.pen_A(255, 255, 255, 2, 0)
            self.pen_B(255, 255, 255, 2, 0)

            # Clear screen
            self.clear()

            # Teleport followers
            leader_init = rospy.get_param("/b01001406Leader_initial_pose")
            A_init = rospy.get_param("/b01001406FollowerA_initial_pose")
            B_init = rospy.get_param("/b01001406FollowerB_initial_pose")

            self.teleport_A(A_init[0], A_init[1], A_init[2])
            self.teleport_B(B_init[0], B_init[1], B_init[2])

            rospy.sleep(0.5)

            # Leader to centre
            resp = self.go_to_goal_client(5.5, 5.5, 0.05, "b01001406Leader")

            if not resp.success:
                rospy.logwarn("GoToGoal failed. Teleporting leader.")
                self.teleport_leader(leader_init[0], leader_init[1], leader_init[2])

            # Face forward
            self.rotate_to_zero()

            # NOW clear the screen (everyone stationary)
            self.clear()

            # Restore original pen colours
            self.pen_leader(255, 0, 0, 3, 0)
            self.pen_A(0, 0, 255, 3, 0)
            self.pen_B(0, 150, 0, 3, 0)

            self.spin_start_time = rospy.Time.now()
            self.state = "SPIN"

            # send spin instruction to followers
            msg = B01001406LeaderMessage()
            msg.instructionID = rospy.Time.now().secs
            msg.message = "SPIN"
            self.pub.publish(msg)

        except Exception as e:
            rospy.logerr(f"Reset failed: {e}")
            self.state = "FORMING"

    def spin_leader(self):

        spin_duration = 1.6  # seconds
        spin_speed = 4.0     # rad/s

        if (rospy.Time.now() - self.spin_start_time).to_sec() < spin_duration:
            vel = Twist()
            vel.angular.z = spin_speed
            self.vel_pub.publish(vel)
        else:
            self.vel_pub.publish(Twist())  # stop spinning
            rospy.loginfo("Spin complete.")
            self.state = "FORMING"        


if __name__ == "__main__":
    try:
        LeaderController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
