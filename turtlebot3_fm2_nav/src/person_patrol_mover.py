#!/usr/bin/env python3
"""Move a Gazebo person model repeatedly through a configured waypoint list.

ROS parameters are documented in ``ROS_PARAMETERS.md``.
"""

import math

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from tf.transformations import quaternion_from_euler

from nav_validation import require_float, require_nonempty_string


def make_state(name: str, x: float, y: float, z: float, yaw: float) -> ModelState:
    """Create a Gazebo model-state message at the requested planar pose."""
    msg = ModelState()
    msg.model_name = name
    msg.reference_frame = "world"

    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    msg.pose = Pose()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw

    msg.twist = Twist()
    return msg


class PersonPatrolMover:
    """ROS node that moves a Gazebo model along a closed patrol route."""

    def __init__(self) -> None:
        """Read patrol configuration and initialize the Gazebo publisher."""
        self.model_name = require_nonempty_string(
            "~model_name", rospy.get_param("~model_name", "person_target")
        )
        self.speed = require_float("~speed", rospy.get_param("~speed", 0.3), 0.0)
        self.z_fixed = require_float("~z_fixed", rospy.get_param("~z_fixed", 0.0))
        self.rate_hz = require_float(
            "~rate", rospy.get_param("~rate", 20.0), 0.0, minimum_inclusive=False
        )
        self.waypoint_tolerance = require_float(
            "~waypoint_tolerance", rospy.get_param("~waypoint_tolerance", 0.05), 0.0
        )

        raw_wps = rospy.get_param(
            "~waypoints",
            [[1.0, 1.0], [1.0, -1.0], [-1.0, -1.0], [-1.0, 1.0]],
        )
        try:
            self.waypoints = [
                (
                    require_float("~waypoints[{}][0]".format(index), point[0]),
                    require_float("~waypoints[{}][1]".format(index), point[1]),
                )
                for index, point in enumerate(raw_wps)
            ]
        except (IndexError, TypeError) as error:
            raise ValueError(
                "parameter '~waypoints' must contain [x, y] pairs"
            ) from error
        if len(self.waypoints) < 2:
            raise ValueError("parameter '~waypoints' must contain at least two points")

        self.pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        self._target_idx = 0
        self._x, self._y = self.waypoints[0]

        rospy.loginfo(
            "Person patrol initialized: model=%s, speed=%.2f m/s, waypoints=%d",
            self.model_name,
            self.speed,
            len(self.waypoints),
        )

    def spin(self) -> None:
        """Move the model through its waypoint loop until ROS shuts down."""
        rate = rospy.Rate(self.rate_hz)
        dt = 1.0 / self.rate_hz
        rospy.sleep(1.0)

        while not rospy.is_shutdown():
            tx, ty = self.waypoints[self._target_idx]
            dx = tx - self._x
            dy = ty - self._y
            dist = math.hypot(dx, dy)

            if dist < self.waypoint_tolerance:
                self._target_idx = (self._target_idx + 1) % len(self.waypoints)
            else:
                step = min(self.speed * dt, dist)
                yaw = math.atan2(dy, dx)
                self._x += math.cos(yaw) * step
                self._y += math.sin(yaw) * step
                self.pub.publish(
                    make_state(self.model_name, self._x, self._y, self.z_fixed, yaw)
                )

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("person_patrol_mover")
    node = PersonPatrolMover()
    node.spin()
