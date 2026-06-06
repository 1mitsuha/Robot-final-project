#!/usr/bin/env python3
"""
ros_compat.py — ROS1 / ROS2 dual-compatibility layer

Detects the ROS version at import time and exposes a unified API so that
the same script runs unchanged on both ROS1 (Noetic) and ROS2 (Humble+).

Usage in every script:
    from ros_compat import (
        CompatNode, spin, ok, shutdown, now, Duration, Time,
        wait_for_message, logerr_throttle,
        # message types also re-exported for convenience
        Point, PoseStamped, Twist, Odometry, OccupancyGrid, Path,
        Float32MultiArray, Marker, MarkerArray, TransformStamped,
        ModelStates, SpawnModel,
    )
"""

import os
import sys

# ---------------------------------------------------------------------------
# Detect ROS version
# 1. Check ROS_VERSION env var first (most reliable, set by setup.bash)
# 2. Fall back to import detection
# ---------------------------------------------------------------------------
_ros_version_env = os.environ.get('ROS_VERSION', '')
if _ros_version_env == '2':
    _ROS2 = True
elif _ros_version_env == '1':
    _ROS2 = False
else:
    try:
        import rclpy  # noqa: F401
        _ROS2 = True
    except ImportError:
        _ROS2 = False

# ---------------------------------------------------------------------------
# Message / service types — identical names across ROS1 & ROS2
# ---------------------------------------------------------------------------
from nav_msgs.msg import OccupancyGrid, Odometry, Path  # noqa: E402
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, TransformStamped  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402
from std_msgs.msg import Float32MultiArray  # noqa: E402
from gazebo_msgs.msg import ModelStates  # noqa: E402

if _ROS2:
    # ROS2: uses SpawnEntity (not SpawnModel)
    from gazebo_msgs.srv import SpawnEntity  # noqa: E402
    from gazebo_msgs.srv import SpawnModel  # noqa: E402  # keep for compat
else:
    from gazebo_msgs.srv import SpawnModel  # noqa: E402
    from gazebo_msgs.srv import SpawnModelRequest  # noqa: E402

if _ROS2:
    import rclpy  # noqa: E402
    import tf2_ros  # noqa: E402
    from rclpy.duration import Duration  # noqa: E402
    from rclpy.time import Time as _RclpyTime  # noqa: E402

    class Time:
        """Unified Time type — wraps rclpy.time.Time in ROS2, rospy.Time in ROS1."""
        def __init__(self, secs=0, nsecs=0):
            self._t = _RclpyTime(seconds=secs, nanoseconds=nsecs)

        @staticmethod
        def from_msg(msg):
            return Time(msg.sec, msg.nanosec)

        def to_msg(self):
            import builtin_interfaces.msg
            return builtin_interfaces.msg.Time(sec=int(self._t.seconds_nanoseconds()[0]),
                                                nanosec=int(self._t.seconds_nanoseconds()[1]))

        def to_sec(self):
            s, ns = self._t.seconds_nanoseconds()
            return float(s) + float(ns) * 1e-9

        def __sub__(self, other):
            if isinstance(other, Time):
                diff = (self._t.nanoseconds - other._t.nanoseconds) * 1e-9
                return Duration(seconds=diff)
            raise TypeError

else:
    import rospy  # noqa: E402
    import tf2_ros  # noqa: E402
    from rospy import Duration, Time  # noqa: E402


# ---------------------------------------------------------------------------
# Spawn service — different names and types in ROS1 vs ROS2
# ---------------------------------------------------------------------------
if _ROS2:
    _SPAWN_SERVICE_NAME = '/spawn_entity'
    _SpawnServiceType = SpawnEntity

    def call_spawn_service(client, model_name, model_xml, robot_namespace,
                           initial_pose, reference_frame, node=None):
        """Call spawn service synchronously (handles ROS2 async internally)."""
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = model_xml
        req.robot_namespace = robot_namespace
        req.initial_pose = initial_pose
        req.reference_frame = reference_frame

        if hasattr(client, 'call_async'):
            # ROS2 async client — spin node until done
            if node is None:
                raise RuntimeError("call_spawn_service requires node=CompatNode in ROS2")
            future = client.call_async(req)
            while rclpy.ok() and not future.done():
                rclpy.spin_once(node._node, timeout_sec=0.01)
            return future.result()
        else:
            # ROS1 sync proxy
            return client(req)

else:
    _SPAWN_SERVICE_NAME = '/gazebo/spawn_sdf_model'
    _SpawnServiceType = SpawnModel

    def call_spawn_service(client, model_name, model_xml, robot_namespace,
                           initial_pose, reference_frame, node=None):
        """Call spawn service synchronously."""
        req = SpawnModelRequest()
        req.model_name = model_name
        req.model_xml = model_xml
        req.robot_namespace = robot_namespace
        req.initial_pose = initial_pose
        req.reference_frame = reference_frame
        return client(req)

def get_spawn_service_name():
    """Return the correct spawn service name for the current ROS version."""
    return _SPAWN_SERVICE_NAME


def get_spawn_service_type():
    """Return the correct spawn service type for the current ROS version."""
    return _SpawnServiceType


# ---------------------------------------------------------------------------
# Core compatibility node
# ---------------------------------------------------------------------------
class CompatNode:
    """
    Unified node wrapper.

    ROS1  — delegates to global ``rospy`` functions.
    ROS2  — wraps a real ``rclpy.node.Node``.
    """

    def __init__(self, name, anonymous=False):
        self._name = name
        if _ROS2:
            if not rclpy.ok():
                rclpy.init(args=sys.argv)
            self._node = rclpy.create_node(name)
        else:
            rospy.init_node(name, anonymous=anonymous)
            self._node = None

    # -- rospy / rclpy global helpers exposed on the instance ---------------
    @staticmethod
    def spin():
        """Block until shutdown."""
        if _ROS2:
            rclpy.spin(_active_node_ref)
        else:
            rospy.spin()

    @staticmethod
    def ok():
        """Return True while the system is running."""
        if _ROS2:
            return rclpy.ok()
        else:
            return not rospy.is_shutdown()

    @staticmethod
    def shutdown():
        if _ROS2:
            rclpy.shutdown()
        else:
            rospy.signal_shutdown("done")

    @staticmethod
    def sleep(duration_sec):
        if _ROS2:
            # non-blocking-ish: spin_once with timeout
            rclpy.spin_once(_active_node_ref, timeout_sec=duration_sec)
        else:
            rospy.sleep(duration_sec)

    # -- publishers / subscribers -------------------------------------------
    def create_publisher(self, msg_type, topic, queue_size=10):
        if _ROS2:
            return self._node.create_publisher(msg_type, topic, queue_size)
        else:
            return rospy.Publisher(topic, msg_type, queue_size=queue_size)

    def create_subscriber(self, msg_type, topic, callback, queue_size=10):
        if _ROS2:
            return self._node.create_subscription(
                msg_type, topic, callback, queue_size)
        else:
            return rospy.Subscriber(
                topic, msg_type, callback, queue_size=queue_size)

    # -- clock --------------------------------------------------------------
    def get_clock(self):
        """Returns an object with a .now() method."""
        return self  # self.now() works on both versions

    def now(self):
        """Return current time as ROS Time / rclpy Time depending on version."""
        if _ROS2:
            return self._node.get_clock().now()
        else:
            return rospy.Time.now()

    # -- rate ---------------------------------------------------------------
    def create_rate(self, hz):
        if _ROS2:
            return _Ros2Rate(self._node, hz)
        else:
            return rospy.Rate(hz)

    # -- logging ------------------------------------------------------------
    def get_logger(self):
        """Return a logger-like object with .info/.warn/.error methods."""
        if _ROS2:
            return self._node.get_logger()
        else:
            return _RospyLogger()

    def loginfo(self, fmt, *args):
        if _ROS2:
            self._node.get_logger().info(fmt % args if args else fmt)
        else:
            rospy.loginfo(fmt, *args)

    def logwarn(self, fmt, *args):
        if _ROS2:
            self._node.get_logger().warn(fmt % args if args else fmt)
        else:
            rospy.logwarn(fmt, *args)

    def logerr(self, fmt, *args):
        if _ROS2:
            self._node.get_logger().error(fmt % args if args else fmt)
        else:
            rospy.logerr(fmt, *args)

    # -- parameters ---------------------------------------------------------
    def get_parameter(self, name, default=None):
        if _ROS2:
            try:
                p = self._node.get_parameter(name)
                return p.value
            except Exception:
                return default
        else:
            # ros1 private param: ~foo → _name/foo
            if name.startswith('~'):
                name = '/' + self._name + '/' + name[1:]
            return rospy.get_param(name, default)

    def declare_parameter(self, name, value):
        if _ROS2:
            self._node.declare_parameter(name, value)
        # ROS1: params set via launch / rosparam, no explicit declare needed

    # -- services -----------------------------------------------------------
    def create_client(self, srv_type, service_name):
        if _ROS2:
            return self._node.create_client(srv_type, service_name)
        else:
            return rospy.ServiceProxy(service_name, srv_type)

    def wait_for_service(self, service_name, timeout=None):
        if _ROS2:
            dummy = self._node.create_client(
                _SpawnServiceType, service_name)
            if timeout is not None:
                return dummy.wait_for_service(timeout_sec=timeout)
            else:
                while not dummy.wait_for_service(timeout_sec=1.0):
                    self.loginfo("Waiting for %s ...", service_name)
                return True
        else:
            if timeout is not None:
                return rospy.wait_for_service(service_name, timeout=timeout)
            else:
                rospy.wait_for_service(service_name)
                return True

    # -- timers -------------------------------------------------------------
    def create_timer(self, period_sec, callback):
        if _ROS2:
            return self._node.create_timer(period_sec, callback)
        else:
            return rospy.Timer(
                rospy.Duration(period_sec),
                lambda event: callback())

    # -- transform broadcaster ----------------------------------------------
    def create_transform_broadcaster(self):
        if _ROS2:
            return tf2_ros.TransformBroadcaster(self._node)
        else:
            return tf2_ros.TransformBroadcaster()

    # -- cross-version helpers ------------------------------------------------
    def stamp_header(self, header):
        """Set header.stamp to now() — works across ROS1/ROS2."""
        t = self.now()
        if _ROS2:
            header.stamp = t.to_msg()
        else:
            header.stamp = t

    def time_diff_sec(self, t1, t2):
        """Return t1 - t2 in seconds (float)."""
        if _ROS2:
            return (t1.nanoseconds - t2.nanoseconds) * 1e-9
        else:
            return (t1 - t2).to_sec()

    def destroy_node(self):
        if _ROS2:
            self._node.destroy_node()


class _Ros2Rate:
    """ROS2 Rate wrapper that also spins callbacks (mimics ROS1's rospy.Rate).

    IMPORTANT: We do NOT use node.create_rate() / Rate.sleep() here, because
    Rate.sleep() blocks on a threading.Event that is only set by a ROS2 Timer
    callback — which requires the executor to be spinning.  That creates a
    chicken-and-egg deadlock when the same thread is responsible for both
    spinning and sleeping.  Instead we spin the executor in a tight loop for
    the duration of one period, which guarantees that timers and subscriptions
    are serviced promptly.
    """
    def __init__(self, node, hz):
        self._node = node
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(node)
        self._period_sec = 1.0 / hz

    def sleep(self):
        # Spin the executor for one period, processing every ready callback.
        # This naturally approximates the desired rate while keeping all
        # subscriptions, timers and services alive.
        import time
        deadline = time.monotonic() + self._period_sec
        while time.monotonic() < deadline:
            self._executor.spin_once(timeout_sec=0.001)


class _RospyLogger:
    """Minimal logger shim that delegates to rospy.log*."""
    def info(self, msg):
        rospy.loginfo(msg)

    def warn(self, msg):
        rospy.logwarn(msg)

    def error(self, msg):
        rospy.logerr(msg)


# ---------------------------------------------------------------------------
# Module-level convenience wrappers
# ---------------------------------------------------------------------------
_active_node_ref = None  # set by each script's main() after creating the node


def spin(node):
    """Block until shutdown. *node* is a CompatNode instance."""
    global _active_node_ref
    _active_node_ref = node
    if _ROS2:
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node._node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
        executor.remove_node(node._node)
    else:
        rospy.spin()


def ok():
    if _ROS2:
        return rclpy.ok()
    else:
        return not rospy.is_shutdown()


def shutdown():
    if _ROS2:
        rclpy.shutdown()
    else:
        rospy.signal_shutdown("done")


def now():
    """Convenience — returns current time. Prefer node.now()."""
    if _ROS2:
        if _active_node_ref is not None:
            return _active_node_ref.now()
        raise RuntimeError("No active node; call node.now() instead.")
    else:
        return rospy.Time.now()


# ---------------------------------------------------------------------------
# wait_for_message
# ---------------------------------------------------------------------------
def wait_for_message(topic, msg_type, node=None):
    """
    Block until a message arrives on *topic*, then return it.
    ROS1: rospy.wait_for_message
    ROS2: spin with a one-shot subscription
    """
    if _ROS2:
        if node is None:
            raise RuntimeError("ROS2 wait_for_message requires a CompatNode")
        result = [None]

        def _cb(msg):
            result[0] = msg

        sub = node._node.create_subscription(
            msg_type, topic, _cb, 10)
        while rclpy.ok() and result[0] is None:
            rclpy.spin_once(node._node, timeout_sec=0.05)
        node._node.destroy_subscription(sub)
        return result[0]
    else:
        return rospy.wait_for_message(topic, msg_type)


# ---------------------------------------------------------------------------
# logerr_throttle
# ---------------------------------------------------------------------------
_throttle_last = {}

def logerr_throttle(period_sec, msg, node=None):
    """ROS1-style logerr_throttle. *node* is required in ROS2."""
    if _ROS2:
        if node is None:
            raise RuntimeError("ROS2 logerr_throttle requires a CompatNode")
        _now = node.now()
        now_sec = _now.nanoseconds * 1e-9
    else:
        now_sec = rospy.Time.now().to_sec()

    key = msg[:80]  # simple throttle key
    last = _throttle_last.get(key, 0.0)
    if now_sec - last >= period_sec:
        _throttle_last[key] = now_sec
        if _ROS2:
            node.get_logger().error(msg)
        else:
            rospy.logerr(msg)
