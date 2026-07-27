#!/usr/bin/env python3
"""Maintain and publish a two-dog shared moving-track knowledge base."""
import json
import os
import sys
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from shared_tracks import SharedTrackKnowledgeBase, build_transform  # noqa: E402


class SharedTrackFusionNode(Node):
    def __init__(self):
        super().__init__("shared_track_fusion_node")
        self.declare_parameter("local_source", os.getenv("DOG_ID", "dog"))
        self.declare_parameter("remote_source", os.getenv("PEER_DOG_ID", "peer"))
        self.declare_parameter(
            "local_to_shared", os.getenv("LOCAL_TO_SHARED", "0,0,0,0"))
        self.declare_parameter(
            "remote_to_shared", os.getenv("REMOTE_TO_SHARED", "0,0,0,0"))
        self.declare_parameter("max_position_distance", 2.0)
        self.declare_parameter("velocity_weight", 0.35)
        self.declare_parameter("min_match_observations", 2)
        self.declare_parameter("max_time_delta", 1.5)

        local_source = self.get_parameter("local_source").value
        remote_source = self.get_parameter("remote_source").value
        if not local_source or not remote_source or local_source == remote_source:
            raise ValueError("local_source and remote_source must be unique")
        self._knowledge_base = SharedTrackKnowledgeBase(
            {
                local_source: build_transform(
                    self.get_parameter("local_to_shared").value),
                remote_source: build_transform(
                    self.get_parameter("remote_to_shared").value),
            },
            max_position_distance=self.get_parameter(
                "max_position_distance").value,
            velocity_weight=self.get_parameter("velocity_weight").value,
            min_match_observations=self.get_parameter(
                "min_match_observations").value,
            max_time_delta=self.get_parameter("max_time_delta").value,
        )
        self._publisher = self.create_publisher(
            String, "/shared_perception/tracks", 10)
        self.create_subscription(
            String,
            "/online_perception/track_observations",
            self._packet_cb,
            10,
        )
        self.create_subscription(
            String,
            "/shared_perception/remote_track_observations",
            self._packet_cb,
            10,
        )
        self.get_logger().info(
            f"shared track fusion ready for {local_source!r} + {remote_source!r}")

    def _packet_cb(self, msg):
        try:
            snapshot = self._knowledge_base.update(json.loads(msg.data))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"ignored invalid track packet: {exc}")
            return
        self._publisher.publish(
            String(data=json.dumps(snapshot, separators=(",", ":"))))


def main():
    rclpy.init()
    node = SharedTrackFusionNode()
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
