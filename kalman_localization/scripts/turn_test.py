#!/usr/bin/env python3

import math
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

TARGET_DEG = 90.0
ANGULAR_SPEED = 0.80
TOLERANCE_DEG = 1.0
NUM_TURNS = 4

current_yaw = None

def odom_cb(msg):
    global current_yaw
    q = msg.pose.pose.orientation
    _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
    current_yaw = yaw

def normalize(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a

rospy.init_node("turn_test")

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rospy.Subscriber("/husky_velocity_controller/odom", Odometry, odom_cb)

rate = rospy.Rate(20)

while current_yaw is None and not rospy.is_shutdown():
    rate.sleep()

target = math.radians(TARGET_DEG)
tol = math.radians(TOLERANCE_DEG)

cmd = Twist()
cmd.angular.z = ANGULAR_SPEED

print("\n==============================")
print(" TESTE DE 4 GIROS")
print("==============================")

for turn in range(NUM_TURNS):

    start_yaw = current_yaw

    print("\n------------------------------")
    print("Giro %d" % (turn + 1))
    print("------------------------------")
    print("Yaw inicial: %.2f°" % math.degrees(start_yaw))

    while not rospy.is_shutdown():

        diff = normalize(current_yaw - start_yaw)
        turned = abs(diff)

        pub.publish(cmd)

        if turned >= target - tol:
            break

        rate.sleep()

    pub.publish(Twist())

    rospy.sleep(1.0)

    final = current_yaw

    print("Yaw final  : %.2f°" % math.degrees(final))
    print("Girou      : %.2f°" % math.degrees(abs(normalize(final-start_yaw))))
    print("Erro       : %.2f°" % (math.degrees(abs(normalize(final-start_yaw)))-TARGET_DEG))

print("\n==============================")
print("FIM DO TESTE")
print("==============================")

pub.publish(Twist())
