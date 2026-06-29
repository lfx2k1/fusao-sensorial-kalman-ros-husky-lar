#!/usr/bin/env python3
"""
Publica /cmd_vel para fazer o Husky percorrer um quadrado (trajetória
repetível), garantindo que os 3 testes (odom / odom+imu / odom+imu+gps)
sejam comparáveis entre si.

O lado reto é feito por tempo fixo. O giro é feito por FEEDBACK real do
yaw (lido da odometria do husky_velocity_controller), não por tempo --
isso evita que pequenos erros de giro (atrito, inercia, rampa de
aceleracao) se acumulem ao longo das voltas e desviem o robo da rota,
o que antes causava colisao com as paredes.
"""
import math
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SIDE_TIME = 6.0       # segundos andando em linha reta
LINEAR_SPEED = 0.5    # m/s
ANGULAR_SPEED = 0.80
TURN_TARGET = math.pi / 2          # 90 graus, em rad
TURN_TOLERANCE = math.radians(1.0) # margem de 1 grau
TURN_TIMEOUT = 6.0    # segundos -- seguranca, evita loop infinito
LAPS = 2

current_yaw = None


def odom_cb(msg):
    global current_yaw
    q = msg.pose.pose.orientation
    _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
    current_yaw = yaw


def angle_diff(a, b):
    """Diferenca normalizada entre dois angulos, no intervalo (-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d <= -math.pi:
        d += 2 * math.pi
    return d


def run():
    global current_yaw
    rospy.init_node('drive_pattern')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/husky_velocity_controller/odom', Odometry, odom_cb)
    rate = rospy.Rate(10)
    rospy.sleep(2.0)  # tempo para tudo subir (gazebo, ekf, etc.)

    while current_yaw is None and not rospy.is_shutdown():
        rospy.loginfo_throttle(1.0, "drive_pattern: esperando odometria...")
        rate.sleep()

    fwd = Twist()
    fwd.linear.x = LINEAR_SPEED
    turn = Twist()
    turn.angular.z = ANGULAR_SPEED
    stop = Twist()

    for lap in range(LAPS):
        for side in range(4):
            # --- lado reto, por tempo (como antes) ---
            t0 = rospy.Time.now()
            while (rospy.Time.now() - t0).to_sec() < SIDE_TIME and not rospy.is_shutdown():
                pub.publish(fwd)
                rate.sleep()
            pub.publish(stop)
            rospy.sleep(0.3)  # deixa o robo estabilizar antes de medir o yaw

            # --- giro de 90 graus, por feedback real do yaw ---
            start_yaw = current_yaw
            t0 = rospy.Time.now()
            while not rospy.is_shutdown():
                turned = math.atan2(
                    math.sin(current_yaw - start_yaw),
                    math.cos(current_yaw - start_yaw)
                )
                turned = abs(turned)
                if turned >= (TURN_TARGET - TURN_TOLERANCE):
                    break
                if (rospy.Time.now() - t0).to_sec() > TURN_TIMEOUT:
                    rospy.logwarn("drive_pattern: timeout no giro (lap=%d side=%d), seguindo assim mesmo", lap, side)
                    break
                pub.publish(turn)
                rate.sleep()
            pub.publish(stop)
            rospy.sleep(0.3)

    pub.publish(stop)
    rospy.loginfo("drive_pattern: trajetoria concluida")


if __name__ == '__main__':
    run()
