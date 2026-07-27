#!/usr/bin/env python3
"""Exchange authenticated rich-track snapshots with one peer over UDP.

DDS remains restricted to the local dog's wired interface so raw LiDAR and
Unitree topics cannot leak onto shared Wi-Fi. Only the small JSON track packet
is sent explicitly to the configured peer address.
"""
import json
import os
import socket
import sys
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from shared_tracks import (  # noqa: E402
    MAX_DATAGRAM_BYTES,
    decode_signed_packet,
    encode_signed_packet,
    validate_packet,
)


class TrackExchangeNode(Node):
    def __init__(self):
        super().__init__("track_exchange_node")
        self.declare_parameter(
            "enabled", os.getenv("TRACK_EXCHANGE_ENABLED", "false").lower() == "true")
        self.declare_parameter("dog_id", os.getenv("DOG_ID", "dog"))
        self.declare_parameter("peer_host", os.getenv("TRACK_PEER_HOST", ""))
        self.declare_parameter(
            "port", int(os.getenv("TRACK_EXCHANGE_PORT", "47020")))
        self.declare_parameter(
            "shared_secret", os.getenv("TRACK_SHARED_SECRET", ""))

        self._enabled = self.get_parameter("enabled").value
        self._dog_id = self.get_parameter("dog_id").value
        self._peer_host = self.get_parameter("peer_host").value
        self._port = self.get_parameter("port").value
        self._secret = self.get_parameter("shared_secret").value
        self._socket = None
        self._received = 0

        self._remote_publisher = self.create_publisher(
            String, "/shared_perception/remote_track_observations", 10)
        self.create_subscription(
            String,
            "/online_perception/track_observations",
            self._local_packet_cb,
            10,
        )

        if self._enabled:
            if not self._dog_id or not self._peer_host or not self._secret:
                raise ValueError(
                    "enabled exchange requires DOG_ID, TRACK_PEER_HOST, "
                    "and TRACK_SHARED_SECRET")
            if not 1 <= self._port <= 65535:
                raise ValueError("TRACK_EXCHANGE_PORT must be 1..65535")
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.bind(("0.0.0.0", self._port))
            self._socket.setblocking(False)
            self.create_timer(0.05, self._receive_cb)

        self.get_logger().warning(
            "track exchange enabled=%s peer=%s:%d"
            % (self._enabled, self._peer_host or "(none)", self._port))

    def _local_packet_cb(self, msg):
        if not self._enabled:
            return
        try:
            packet = validate_packet(json.loads(msg.data))
            if packet["source_id"] != self._dog_id:
                raise ValueError(
                    "local packet source_id does not match configured DOG_ID")
            encoded = encode_signed_packet(packet, self._secret)
            self._socket.sendto(encoded, (self._peer_host, self._port))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"local track packet not sent: {exc}")
        except OSError as exc:
            self.get_logger().warning(f"track packet send failed: {exc}")

    def _receive_cb(self):
        # Bound work per timer tick so a burst cannot starve ROS callbacks.
        for _ in range(20):
            try:
                data, address = self._socket.recvfrom(MAX_DATAGRAM_BYTES + 1)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().warning(f"track packet receive failed: {exc}")
                return
            try:
                packet = decode_signed_packet(data, self._secret)
                if packet["source_id"] == self._dog_id:
                    raise ValueError("ignored packet claiming this dog's source_id")
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                self.get_logger().warning(
                    f"rejected track packet from {address[0]}: {exc}")
                continue
            self._remote_publisher.publish(
                String(data=json.dumps(packet, separators=(",", ":"))))
            self._received += 1
            if self._received % 20 == 0:
                self.get_logger().info(
                    f"received {self._received} authenticated track packets")

    def destroy_node(self):
        if self._socket is not None:
            self._socket.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TrackExchangeNode()
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
