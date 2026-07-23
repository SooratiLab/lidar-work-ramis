"""Turn confirmed nearby tracks into a debounced stop request.

Subscribes to `/online_perception/tracks` and FastLIO `/Odometry`, then
publishes `std_msgs/Bool` on `/online_perception/stop_requested`. This node
does not command the Go2 directly. A robot-specific adapter should consume
the request only after the detector and CycloneDDS path have been validated
on hardware; keeping that boundary explicit makes bag replay safe.
"""
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parent))
from response_policy import NearClusterStopPolicy  # noqa: E402


class ClusterResponseNode(Node):
    def __init__(self):
        super().__init__("cluster_response_node")
        self.declare_parameter("stop_distance", 2.0)
        self.declare_parameter("clear_distance", 2.5)
        self.declare_parameter("trigger_frames", 2)
        self.declare_parameter("clear_frames", 2)
        self._policy = NearClusterStopPolicy(
            self.get_parameter("stop_distance").value,
            self.get_parameter("clear_distance").value,
            self.get_parameter("trigger_frames").value,
            self.get_parameter("clear_frames").value,
        )
        self._sensor_position = None
        self._publisher = self.create_publisher(
            Bool, "/online_perception/stop_requested", 10)
        self.create_subscription(Odometry, "/Odometry", self._odom_cb, 10)
        self.create_subscription(
            PoseArray, "/online_perception/tracks", self._tracks_cb, 10)
        self.get_logger().info(
            "cluster response ready -- publishing stop requests only "
            "(no Go2 actuation)")

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self._sensor_position = np.array([p.x, p.y, p.z])

    def _tracks_cb(self, msg):
        if self._sensor_position is None:
            self.get_logger().warning(
                "tracks received before /Odometry; no response evaluated",
                throttle_duration_sec=5.0)
            return
        clusters = [[p.position.x, p.position.y, p.position.z]
                    for p in msg.poses]
        decision = self._policy.update(clusters, self._sensor_position)
        self._publisher.publish(Bool(data=decision.stop_requested))
        if decision.changed:
            distance = ("none" if decision.nearest_distance is None
                        else f"{decision.nearest_distance:.2f} m")
            state = "STOP REQUESTED" if decision.stop_requested else "stop request cleared"
            self.get_logger().warning(f"{state}; nearest confirmed cluster: {distance}")


def main():
    rclpy.init()
    node = ClusterResponseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
