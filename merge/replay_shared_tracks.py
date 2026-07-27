#!/usr/bin/env python3
"""Replay two offline track CSVs through the live shared-track fusion core.

This is a transport-free test: it exercises timestamps, transforms,
association persistence, and shared snapshots without pretending that a CSV
can validate Wi-Fi packet loss or ROS scheduling. CSV exports do not contain
cluster extents or velocity vectors, so velocity is estimated from consecutive
centroids and size is left unknown.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from shared_tracks import SharedTrackKnowledgeBase, build_transform


def load_packets(path, source_id, time_offset=0.0):
    rows_by_frame = defaultdict(list)
    with Path(path).open(newline="") as handle:
        for row in csv.DictReader(handle):
            if row["status"] == "coasting":
                continue
            rows_by_frame[int(row["frame"])].append(row)

    previous = {}
    packets = []
    for sequence, frame in enumerate(sorted(rows_by_frame)):
        tracks = []
        frame_rows = rows_by_frame[frame]
        stamp = float(frame_rows[0]["time_s"]) + time_offset
        for row in frame_rows:
            local_id = int(row["track_id"])
            position = np.array([
                float(row["centroid_x_m"]),
                float(row["centroid_y_m"]),
                float(row["centroid_z_m"]),
            ])
            velocity = np.zeros(3)
            if local_id in previous:
                old_stamp, old_position = previous[local_id]
                elapsed = stamp - old_stamp
                if elapsed > 0:
                    velocity = (position - old_position) / elapsed
            previous[local_id] = (stamp, position)
            tracks.append({
                "local_id": local_id,
                "position_m": position.tolist(),
                "velocity_m_s": velocity.tolist(),
                "size_m": [0.0, 0.0, 0.0],
                "n_points": int(row["n_points"]),
            })
        packets.append({
            "schema_version": 1,
            "source_id": source_id,
            "session_id": f"{source_id}-csv-replay",
            "sequence": sequence,
            "stamp_s": stamp,
            "frame_id": f"{source_id}/offline",
            "sensor_position_m": [0.0, 0.0, 0.0],
            "tracks": tracks,
        })
    return packets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dog1_tracks", type=Path)
    parser.add_argument("dog2_tracks", type=Path)
    parser.add_argument(
        "--dog2-to-dog1", default="0,0,0,0",
        help="dog 2 -> dog 1 transform as x_m,y_m,z_m,yaw_deg")
    parser.add_argument(
        "--dog2-time-offset", type=float, default=0.0,
        help="seconds added to dog 2's CSV timestamps")
    parser.add_argument("--output", type=Path, help="optional snapshot JSONL")
    args = parser.parse_args()

    knowledge_base = SharedTrackKnowledgeBase({
        "dog1": np.eye(4),
        "dog2": build_transform(args.dog2_to_dog1),
    })
    events = load_packets(args.dog1_tracks, "dog1") + load_packets(
        args.dog2_tracks, "dog2", args.dog2_time_offset)
    # On equal timestamps dog1 is applied first, giving each pair the same
    # deterministic callback ordering as a typical live run.
    events.sort(key=lambda item: (item["stamp_s"], item["source_id"]))

    snapshots = []
    associated_ids = set()
    for event in events:
        snapshot = knowledge_base.update(event)
        snapshots.append(snapshot)
        associated_ids.update(
            item["global_id"] for item in snapshot["tracks"]
            if len(item["contributors"]) == 2
        )

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w") as handle:
            for snapshot in snapshots:
                handle.write(json.dumps(snapshot, separators=(",", ":")) + "\n")

    print(f"processed packets: {len(events)}")
    snapshots_with_association = sum(
        any(len(track["contributors"]) == 2 for track in snapshot["tracks"])
        for snapshot in snapshots
    )
    print("snapshots containing a shared association: "
          f"{snapshots_with_association}")
    print(f"distinct associated ID pairs: {len(associated_ids)}")
    for global_id in sorted(associated_ids):
        print(f"  {global_id}")
    if args.output:
        print(f"snapshot log: {args.output}")


if __name__ == "__main__":
    main()
