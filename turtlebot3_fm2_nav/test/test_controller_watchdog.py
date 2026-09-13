#!/usr/bin/env python3
"""Integration test for controller motion and explicit route cancellation."""

import unittest
from typing import Callable

import rospy
import rostest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path


class ControllerWatchdogTest(unittest.TestCase):
    """Ensure a valid route commands motion and an empty route commands stop."""

    def setUp(self) -> None:
        """Create test publishers and wait until the controller subscribes."""
        self.commands = []
        self.command_sub = rospy.Subscriber("/cmd_vel", Twist, self.commands.append)
        self.pose_pub = rospy.Publisher(
            "/amcl_pose", PoseWithCovarianceStamped, queue_size=1
        )
        self.path_pub = rospy.Publisher("/fm2_path", Path, queue_size=1)
        self._wait_for_connections()

    def _wait_for_connections(self) -> None:
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if (
                self.pose_pub.get_num_connections()
                and self.path_pub.get_num_connections()
            ):
                return
            rospy.sleep(0.05)
        self.fail("controller subscriptions were not connected")

    def _wait_for(self, predicate: Callable[[], bool], description: str) -> None:
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return
            rospy.sleep(0.05)
        self.fail("timed out waiting for {}".format(description))

    @staticmethod
    def _pose_message() -> PoseWithCovarianceStamped:
        message = PoseWithCovarianceStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        message.pose.pose.orientation.w = 1.0
        return message

    @staticmethod
    def _path_message() -> Path:
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        for x in (1.0, 2.0):
            point = PoseStamped()
            point.header = path.header
            point.pose.position.x = x
            point.pose.orientation.w = 1.0
            path.poses.append(point)
        return path

    def test_motion_then_stop_on_empty_path(self) -> None:
        """Command motion for a valid path then stop on cancellation."""
        self.pose_pub.publish(self._pose_message())
        self.path_pub.publish(self._path_message())
        self._wait_for(
            lambda: any(command.linear.x > 0.0 for command in self.commands),
            "motion command",
        )

        self.path_pub.publish(Path())
        self._wait_for(
            lambda: bool(self.commands)
            and self.commands[-1].linear.x == 0.0
            and self.commands[-1].angular.z == 0.0,
            "zero command after route cancellation",
        )


if __name__ == "__main__":
    rospy.init_node("test_controller_watchdog")
    rostest.rosrun("turtlebot3_fm2_nav", "controller_watchdog", ControllerWatchdogTest)
