#!/usr/bin/env python3
import math
import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from tf.transformations import quaternion_from_euler


def make_state(name, x, y, z, yaw):
    msg = ModelState()
    msg.model_name = name
    msg.reference_frame = 'world'

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


if __name__ == '__main__':
    rospy.init_node('move_person')

    model_name = rospy.get_param('~model_name', 'person_target')
    x_fixed = rospy.get_param('~x_fixed', 3.0)
    z_fixed = rospy.get_param('~z_fixed', 0.0)
    amplitude = rospy.get_param('~amplitude', 2.0)
    period = rospy.get_param('~period', 8.0)
    rate_hz = rospy.get_param('~rate', 20.0)

    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rate = rospy.Rate(rate_hz)
    start = rospy.Time.now().to_sec()

    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start
        omega = 2.0 * math.pi / period
        y = amplitude * math.sin(omega * t)
        vy = amplitude * omega * math.cos(omega * t)
        yaw = math.pi / 2.0 if vy >= 0.0 else -math.pi / 2.0

        pub.publish(make_state(model_name, x_fixed, y, z_fixed, yaw))
        rate.sleep()
