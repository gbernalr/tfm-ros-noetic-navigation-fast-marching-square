#!/usr/bin/env python3
"""Initialize AMCL with the pose used to spawn the TurtleBot in Gazebo.

ROS parameters are documented in ``ROS_PARAMETERS.md``.
"""

import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_validation import require_float, require_int, require_nonempty_string


def main() -> None:
    """Build and repeatedly publish the configured initial AMCL pose."""
    rospy.init_node("initial_pose_publisher")
    frame_id = require_nonempty_string("~frame_id", rospy.get_param("~frame_id", "map"))
    x = require_float("~x", rospy.get_param("~x", 0.0))
    y = require_float("~y", rospy.get_param("~y", 0.0))
    yaw = require_float("~yaw", rospy.get_param("~yaw", 0.0))
    repeats = require_int("~repeats", rospy.get_param("~repeats", 5), 1)
    rate_hz = require_float(
        "~rate", rospy.get_param("~rate", 1.0), 0.0, minimum_inclusive=False
    )
    std_xy = require_float("~std_xy", rospy.get_param("~std_xy", 0.10), 0.0)
    std_yaw = require_float("~std_yaw", rospy.get_param("~std_yaw", 0.10), 0.0)

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

    # Gazebo, AMCL, and their subscribers start concurrently. Repeating this
    # message avoids losing initialization because of a startup race.
    rate = rospy.Rate(rate_hz)
    for _ in range(repeats):
        if rospy.is_shutdown():
            return
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo(
        "AMCL initial pose published in %s: (%.2f, %.2f, yaw=%.2f rad)",
        frame_id,
        x,
        y,
        yaw,
    )


if __name__ == "__main__":
    main()
