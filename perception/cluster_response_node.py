"""Turn confirmed nearby tracks into a debounced stop request.

Subscribes to `/online_perception/tracks` and FastLIO `/Odometry`, then
publishes `std_msgs/Bool` on `/online_perception/stop_requested` and a JSON
status record on `/online_perception/response_status`. Stale perception or
odometry fails safe to a stop request by default. This node does not command
the Go2 directly; keeping that boundary explicit makes bag replay safe.
"""
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from response_policy import NearClusterStopPolicy  # noqa: E402


class ClusterResponseNode(Node):
    def __init__(self):
        super().__init__("cluster_response_node")
        self.declare_parameter("stop_distance", 2.0)
        self.declare_parameter("clear_distance", 2.5)
        self.declare_parameter("trigger_duration", 1.0)
        self.declare_parameter("clear_duration", 1.0)
        self.declare_parameter("planar_distance", True)
        self.declare_parameter("input_timeout", 2.5)
        self.declare_parameter("odometry_timeout", 2.5)
        self.declare_parameter("fail_safe_stop", True)
        self._policy = NearClusterStopPolicy(
            self.get_parameter("stop_distance").value,
            self.get_parameter("clear_distance").value,
            self.get_parameter("trigger_duration").value,
            self.get_parameter("clear_duration").value,
            self.get_parameter("planar_distance").value,
        )
        self._input_timeout = self.get_parameter("input_timeout").value
        self._odometry_timeout = self.get_parameter("odometry_timeout").value
        self._fail_safe_stop = self.get_parameter("fail_safe_stop").value
        if self._input_timeout <= 0 or self._odometry_timeout <= 0:
            raise ValueError("freshness timeouts must be positive")
        self._sensor_position = None
        self._last_tracks_received = None
        self._last_odom_received = None
        self._last_published_request = None
        self._last_status_state = None
        self._publisher = self.create_publisher(
            Bool, "/online_perception/stop_requested", 10)
        self._status_publisher = self.create_publisher(
            String, "/online_perception/response_status", 10)
        self.create_subscription(Odometry, "/Odometry", self._odom_cb, 10)
        self.create_subscription(
            PoseArray, "/online_perception/tracks", self._tracks_cb, 10)
        self.create_timer(0.25, self._freshness_cb)
        self.get_logger().info(
            "cluster response ready -- %.2f m %s stop threshold, "
            "%.2f s dwell, stale input requests stop=%s (no Go2 actuation)"
            % (
                self._policy.stop_distance,
                "planar" if self._policy.planar_distance else "3D",
                self._policy.trigger_duration,
                self._fail_safe_stop,
            ))

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self._sensor_position = np.array([p.x, p.y, p.z])
        self._last_odom_received = self._now_seconds()

    def _tracks_cb(self, msg):
        now = self._now_seconds()
        self._last_tracks_received = now
        if self._sensor_position is None:
            self._publish_state(
                self._fail_safe_stop, "waiting_for_odometry", None, now)
            return
        clusters = [[p.position.x, p.position.y, p.position.z]
                    for p in msg.poses]
        decision = self._policy.update(clusters, self._sensor_position, now)
        self._publish_state(
            decision.stop_requested,
            decision.state,
            decision.nearest_distance,
            now,
        )
        if decision.changed:
            distance = ("none" if decision.nearest_distance is None
                        else f"{decision.nearest_distance:.2f} m")
            state = "STOP REQUESTED" if decision.stop_requested else "stop request cleared"
            self.get_logger().warning(f"{state}; nearest confirmed cluster: {distance}")

    def _freshness_cb(self):
        now = self._now_seconds()
        if self._last_odom_received is None:
            self._publish_state(
                self._fail_safe_stop, "waiting_for_odometry", None, now)
        elif now - self._last_odom_received > self._odometry_timeout:
            self._publish_state(
                self._fail_safe_stop, "stale_odometry", None, now)
        elif self._last_tracks_received is None:
            self._publish_state(
                self._fail_safe_stop, "waiting_for_tracks", None, now)
        elif now - self._last_tracks_received > self._input_timeout:
            self._publish_state(
                self._fail_safe_stop, "stale_tracks", None, now)

    def _publish_state(self, stop_requested, state, nearest_distance, now):
        self._publisher.publish(Bool(data=stop_requested))
        status = {
            "state": state,
            "stop_requested": stop_requested,
            "nearest_distance_m": nearest_distance,
            "tracks_age_s": (
                None if self._last_tracks_received is None
                else max(0.0, now - self._last_tracks_received)
            ),
            "odometry_age_s": (
                None if self._last_odom_received is None
                else max(0.0, now - self._last_odom_received)
            ),
        }
        self._status_publisher.publish(String(data=json.dumps(status)))
        if (
            stop_requested != self._last_published_request
            or state != self._last_status_state
        ):
            self.get_logger().warning(
                "response state=%s stop_requested=%s" % (state, stop_requested))
        self._last_published_request = stop_requested
        self._last_status_state = state


def main():
    rclpy.init()
    node = ClusterResponseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
