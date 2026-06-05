#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_compat import (
    CompatNode, spin, logerr_throttle,
    ModelStates, TransformStamped,
)


class TFBroadcaster:
    def __init__(self, node):
        self.node = node
        self.broadcaster = node.create_transform_broadcaster()
        self.last_time = self._time_zero()
        self.rate = node.create_rate(30)

    def _time_zero(self):
        """Return epoch time (0, 0) — rospy.Time(0) / rclpy.time.Time()."""
        from ros_compat import Time
        return Time(secs=0, nsecs=0)

    def callback(self, msg):
        try:
            model_name = self.node.get_parameter('~model_name', 'turtlebot3_waffle')
            idx = msg.name.index(model_name)
            current_time = self.node.now()

            # Prevent duplicate timestamps
            if self._time_eq(current_time, self.last_time):
                # add a tiny offset
                current_time = self._time_add(current_time, 0, 1000000)  # +0.001s

            tf_msg = TransformStamped()
            tf_msg.header.stamp = current_time.to_msg()
            tf_msg.header.frame_id = "map"
            tf_msg.child_frame_id = "base_footprint"
            tf_msg.transform.translation = msg.pose[idx].position
            tf_msg.transform.rotation = msg.pose[idx].orientation

            self.broadcaster.sendTransform(tf_msg)
            self.last_time = current_time

        except ValueError as e:
            logerr_throttle(1.0, f"Model {model_name} not found: {str(e)}", node=self.node)

    def _time_eq(self, t1, t2):
        """Check if two times are equal across ROS versions."""
        try:
            # ROS2: rclpy.time.Time
            return t1.nanoseconds == t2.nanoseconds
        except AttributeError:
            # ROS1: rospy.Time
            return t1 == t2

    def _time_add(self, t, secs, nsecs):
        """Add secs + nsecs to a time, cross-version."""
        try:
            # ROS2
            from rclpy.duration import Duration
            from rclpy.time import Time
            d = Duration(seconds=secs, nanoseconds=nsecs)
            return Time(nanoseconds=t.nanoseconds + d.nanoseconds)
        except ImportError:
            # ROS1
            import rospy
            return t + rospy.Duration(secs, nsecs)


if __name__ == '__main__':
    node = CompatNode('gazebo_to_tf')
    tf_broadcaster = TFBroadcaster(node)
    node.create_subscriber(ModelStates, "/gazebo/model_states", tf_broadcaster.callback)
    spin(node)
