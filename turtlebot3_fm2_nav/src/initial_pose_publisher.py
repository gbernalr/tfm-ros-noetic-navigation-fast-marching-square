#!/usr/bin/env python3
"""Inicializa AMCL con la misma pose usada al spawnear el TurtleBot en Gazebo."""

import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    rospy.init_node("initial_pose_publisher")
    frame_id = rospy.get_param("~frame_id", "map")
    x = float(rospy.get_param("~x", 0.0))
    y = float(rospy.get_param("~y", 0.0))
    yaw = float(rospy.get_param("~yaw", 0.0))
    repeats = max(1, int(rospy.get_param("~repeats", 5)))
    rate_hz = max(0.1, float(rospy.get_param("~rate", 1.0)))
    std_xy = float(rospy.get_param("~std_xy", 0.10))
    std_yaw = float(rospy.get_param("~std_yaw", 0.10))

    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
    msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
    msg.pose.covariance[0] = std_xy**2
    msg.pose.covariance[7] = std_xy**2
    msg.pose.covariance[35] = std_yaw**2

    # El spawn de Gazebo, AMCL y sus suscriptores arrancan en paralelo. Repetir
    # el mensaje evita perder la inicialización por una carrera de arranque.
    rate = rospy.Rate(rate_hz)
    for _ in range(repeats):
        if rospy.is_shutdown():
            return
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("AMCL inicializado en map: (%.2f, %.2f, yaw=%.2f rad)", x, y, yaw)


if __name__ == "__main__":
    main()
