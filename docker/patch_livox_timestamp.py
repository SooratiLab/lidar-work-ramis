#!/usr/bin/env python3
"""
Force livox_ros_driver2 to timestamp every point/IMU sample from the host
clock, instead of trusting the sensor's own clock.

The Mid-360 reports its packets as kTimestampTypeGptpOrPtp even when no PTP
master is present anywhere on the network. GetEthPacketTimestamp() in
pub_handler.cpp trusts that flag at face value and returns the sensor's own
free-running clock, which is never synced to the host. Meanwhile IMU samples
in the same driver are timestamped from the host clock. The result is a
20-35 second gap between LiDAR and IMU timestamps, which stops FastLIO from
fusing any data at all -- it never advances past IMU initialisation.

Usage (against a freshly cloned livox_ros_driver2 checkout, before building):
    python3 patch_livox_timestamp.py <path to livox_ros_driver2>
"""
import re
import sys
from pathlib import Path

TARGET_RELATIVE_PATH = "src/comm/pub_handler.cpp"

# Matches the whole GetEthPacketTimestamp function body, from its signature
# to the closing brace at column 0. Written against upstream at
# github.com/Livox-SDK/livox_ros_driver2, master branch.
FUNCTION_PATTERN = re.compile(
    r"uint64_t PubHandler::GetEthPacketTimestamp\([^)]*\)\s*\{.*?\n\}",
    re.DOTALL,
)

REPLACEMENT = (
    "uint64_t PubHandler::GetEthPacketTimestamp(uint8_t timestamp_type, "
    "uint8_t* time_stamp, uint8_t size) {\n"
    "  // Always use the host clock. The Mid-360 reports a PTP/GPS-style\n"
    "  // timestamp type even with no real time source on the network,\n"
    "  // which desyncs LiDAR timestamps from the IMU (host clock) by tens\n"
    "  // of seconds and stops FastLIO from fusing any data.\n"
    "  return std::chrono::high_resolution_clock::now().time_since_epoch().count();\n"
    "}"
)

ALREADY_PATCHED_MARKER = "Always use the host clock."


def patch_file(path: Path) -> None:
    text = path.read_text()

    if ALREADY_PATCHED_MARKER in text:
        print(f"{path}: timestamp patch already applied, nothing to do")
        return

    patched, count = FUNCTION_PATTERN.subn(REPLACEMENT, text, count=1)
    if count != 1:
        sys.exit(
            f"{path}: could not find GetEthPacketTimestamp() to patch -- "
            "upstream source has likely changed shape, check pub_handler.cpp "
            "by hand and update FUNCTION_PATTERN in this script"
        )

    path.write_text(patched)
    print(f"{path}: timestamp patch applied")


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(f"usage: {sys.argv[0]} <livox_ros_driver2 checkout path>")

    target = Path(sys.argv[1]) / TARGET_RELATIVE_PATH
    if not target.is_file():
        sys.exit(f"not found: {target}")

    patch_file(target)


if __name__ == "__main__":
    main()
