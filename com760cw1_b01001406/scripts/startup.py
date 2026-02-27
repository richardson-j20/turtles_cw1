#!/usr/bin/env python3
import random
import rospy
from turtlesim.srv import Spawn, Kill, SetPen
from std_srvs.srv import Empty

rospy.init_node("startup")

rospy.wait_for_service("/kill")
rospy.wait_for_service("/spawn")
rospy.wait_for_service("/clear")

kill = rospy.ServiceProxy("/kill", Kill)
spawn = rospy.ServiceProxy("/spawn", Spawn)
clear = rospy.ServiceProxy("/clear", Empty)

# random background colour
rospy.set_param("/turtlesim/background_r", random.randint(0, 255))
rospy.set_param("/turtlesim/background_g", random.randint(0, 255))
rospy.set_param("/turtlesim/background_b", random.randint(0, 255))
clear()

# remove default turtle
try:
    kill("turtle1")
except Exception:
    pass

# function for follower turtles to appear in a random pose
def random_pose():
    x = random.choice([
        random.uniform(1.0, 4.0),
        random.uniform(7.0, 10.0)
    ])
    y = random.choice([
        random.uniform(1.0, 4.0),
        random.uniform(7.0, 10.0)
    ])
    theta = random.uniform(-3.14, 3.14)
    return x, y, theta

# spawn leader
leader_pose = (5.5, 5.5, 0.0)
spawn(*leader_pose, "b01001406Leader")

# spawn followers
followerA_pose = random_pose()
spawn(*followerA_pose, "b01001406FollowerA")

followerB_pose = random_pose()
spawn(*followerB_pose, "b01001406FollowerB")

# store initial poses
rospy.set_param("/b01001406Leader_initial_pose", leader_pose)
rospy.set_param("/b01001406FollowerA_initial_pose", followerA_pose)
rospy.set_param("/b01001406FollowerB_initial_pose", followerB_pose)

# set pen colour for each turtle so I know which one is which when they move around
# leader is red, follower A is blue and follower B is green
def set_pen_colour(name, r, g, b):
    rospy.wait_for_service(f"/{name}/set_pen")
    pen = rospy.ServiceProxy(f"/{name}/set_pen", SetPen)
    pen(r, g, b, 3, 0)

set_pen_colour("b01001406Leader", 255, 0, 0)
set_pen_colour("b01001406FollowerA", 0, 0, 255)
set_pen_colour("b01001406FollowerB", 0, 150, 0)

rospy.sleep(0.5)
