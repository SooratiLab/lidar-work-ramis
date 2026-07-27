#!/usr/bin/env python3
"""Optional, disabled-by-default adapter from stop requests to Unitree.

Dry-run mode is the default and does not import or publish a Unitree message.
Setting ``enabled:=true`` requires ``unitree_api`` to be installed, publishes
StopMove (API ID 1003) on ``/api/sport/request``, and repeats the request at a
bounded rate while the stop condition remains active. There is intentionally
no automatic resume command: clearing the perception request only re-arms the
adapter; the operator remains responsible for resuming motion.
"""
import json
import sys
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from actuation_policy import StopCommandGate  # noqa: E402


STOP_MOVE_API_ID = 1003


class StopActuationNode(Node):
    def __init__(self):
        super().__init__("stop_actuation_node")
        self.declare_parameter("enabled", False)
        self.declare_parameter("repeat_interval", 1.0)
        self.declare_parameter("request_timeout", 2.5)

        self._enabled = self.get_parameter("enabled").value
        self._request_timeout = self.get_parameter("request_timeout").value
        if self._request_timeout <= 0:
            raise ValueError("request_timeout must be positive")
        self._gate = StopCommandGate(
            self.get_parameter("repeat_interval").value)
        self._last_request_received = None
        self._request_type = None
        self._command_publisher = None

        if self._enabled:
            try:
                from unitree_api.msg import Request
            except ImportError as exc:
                raise RuntimeError(
                    "enabled actuation requires the unitree_api ROS package; "
                    "rebuild the Docker image from the current Dockerfile"
                ) from exc
            self._request_type = Request
            self._command_publisher = self.create_publisher(
                Request, "/api/sport/request", 10)

        self._status_publisher = self.create_publisher(
            String, "/online_perception/actuation_status", 10)
        self.create_subscription(
            Bool, "/online_perception/stop_requested", self._request_cb, 10)
        self.create_timer(0.25, self._watchdog_cb)
        self.get_logger().warning(
            "stop actuation adapter ready: enabled=%s; %s"
            % (
                self._enabled,
                "StopMove commands may be published"
                if self._enabled else "dry run only, no robot commands",
            ))

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _request_cb(self, msg):
        now = self._now_seconds()
        self._last_request_received = now
        self._handle_request(msg.data, now, "response_topic")

    def _watchdog_cb(self):
        now = self._now_seconds()
        if (
            self._last_request_received is None
            or now - self._last_request_received > self._request_timeout
        ):
            # If this adapter is ever enabled while the response node dies,
            # continuing motion is the unsafe interpretation of silence.
            self._handle_request(True, now, "stale_response_topic")

    def _handle_request(self, stop_requested, now, source):
        decision = self._gate.update(stop_requested, now)
        if decision.should_send_stop:
            if self._enabled:
                request = self._request_type()
                request.header.identity.api_id = STOP_MOVE_API_ID
                self._command_publisher.publish(request)
                action = "sent_stop_move"
            else:
                action = "would_send_stop_move"
            self.get_logger().warning("%s (%s)" % (action, source))
        else:
            action = decision.reason

        status = {
            "enabled": self._enabled,
            "stop_requested": bool(stop_requested),
            "action": action,
            "source": source,
        }
        self._status_publisher.publish(String(data=json.dumps(status)))


def main():
    rclpy.init()
    node = StopActuationNode()
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
