#!/usr/bin/env python3
"""
Converte sensor_msgs/NavSatFix (/fix) em nav_msgs/Odometry (/gps/odom),
usando projeção equirretangular local (flat-earth) em torno de uma
referência lat0/lon0 (a mesma referência configurada no plugin GPS do Gazebo).

ATENCAO: aproximação simplificada (válida para áreas pequenas, como a
simulação). Não usar em aplicações reais sem uma projeção geodésica adequada
(UTM, pyproj, etc.) - ponto discutido no README.
"""
import math
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

EARTH_RADIUS = 6378137.0  # raio equatorial WGS84 (m)


class GpsToOdom:
    def __init__(self):
        self.lat0 = math.radians(rospy.get_param('~ref_latitude', 0.0))
        self.lon0 = math.radians(rospy.get_param('~ref_longitude', 0.0))
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        self.pos_cov = float(rospy.get_param('~position_covariance', 1.0))

        self.pub = rospy.Publisher('/gps/odom', Odometry, queue_size=10)
        rospy.Subscriber('/fix', NavSatFix, self.cb, queue_size=10)

    def cb(self, msg):
        if msg.status.status < 0:
            return  # sem fix

        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)

        # Projeção equirretangular local (East/North), alinhada ao frame
        # do mundo do Gazebo (referenceHeading=0 no plugin GPS).
        x = (lon - self.lon0) * EARTH_RADIUS * math.cos(self.lat0)
        y = (lat - self.lat0) * EARTH_RADIUS

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0  # GPS não fornece orientação

        # Covariância: x,y confiáveis; z e orientação "infinitas" (não usadas)
        cov = [0.0] * 36
        cov[0] = self.pos_cov   # x
        cov[7] = self.pos_cov   # y
        cov[14] = 1e6           # z
        cov[21] = 1e6           # roll
        cov[28] = 1e6           # pitch
        cov[35] = 1e6           # yaw
        odom.pose.covariance = cov

        self.pub.publish(odom)


if __name__ == '__main__':
    rospy.init_node('gps_to_odom')
    GpsToOdom()
    rospy.spin()
