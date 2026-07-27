"""Wire format and stateful live fusion for small cross-dog track packets.

Only confirmed tracks backed by a current measurement belong in this
protocol. Coordinates are metres and timestamps are Unix/ROS seconds. Each
source has a configured rigid transform into one shared frame; without that
transform, centroid proximity across dogs is meaningless.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import math

import numpy as np
from scipy.optimize import linear_sum_assignment


SCHEMA_VERSION = 1
MAX_DATAGRAM_BYTES = 60_000


def build_transform(offset: str) -> np.ndarray:
    """Parse ``x_m,y_m,z_m,yaw_deg`` into a 4x4 shared-frame transform."""
    values = [float(value.strip()) for value in offset.split(",")]
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("transform must be x_m,y_m,z_m,yaw_deg")
    x_m, y_m, z_m, yaw_deg = values
    yaw = math.radians(yaw_deg)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    transform = np.eye(4)
    transform[:3, :3] = [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]
    transform[:3, 3] = [x_m, y_m, z_m]
    return transform


def _finite_vector(value, name):
    vector = np.asarray(value, dtype=float).reshape(3)
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain three finite values")
    return vector.tolist()


def validate_packet(packet: dict) -> dict:
    """Validate and normalize one source snapshot before using or forwarding."""
    if packet.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            f"unsupported schema_version {packet.get('schema_version')!r}")
    source_id = packet.get("source_id")
    session_id = packet.get("session_id")
    frame_id = packet.get("frame_id")
    if not isinstance(source_id, str) or not source_id:
        raise ValueError("source_id must be a non-empty string")
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError("frame_id must be a non-empty string")
    if not isinstance(session_id, str) or not session_id:
        raise ValueError("session_id must be a non-empty string")
    sequence = int(packet["sequence"])
    stamp_s = float(packet["stamp_s"])
    if sequence < 0 or not math.isfinite(stamp_s):
        raise ValueError("sequence must be non-negative and stamp_s finite")

    tracks = []
    seen_ids = set()
    for raw_track in packet.get("tracks", []):
        local_id = int(raw_track["local_id"])
        if local_id < 0 or local_id in seen_ids:
            raise ValueError("track local_id values must be unique and non-negative")
        seen_ids.add(local_id)
        n_points = int(raw_track.get("n_points", 0))
        if n_points < 0:
            raise ValueError("n_points must be non-negative")
        size = _finite_vector(
            raw_track.get("size_m", [0, 0, 0]), "size_m")
        if any(value < 0 for value in size):
            raise ValueError("size_m values must be non-negative")
        tracks.append({
            "local_id": local_id,
            "position_m": _finite_vector(raw_track["position_m"], "position_m"),
            "velocity_m_s": _finite_vector(
                raw_track.get("velocity_m_s", [0, 0, 0]), "velocity_m_s"),
            "size_m": size,
            "n_points": n_points,
        })

    return {
        "schema_version": SCHEMA_VERSION,
        "source_id": source_id,
        "session_id": session_id,
        "sequence": sequence,
        "stamp_s": stamp_s,
        "frame_id": frame_id,
        "sensor_position_m": _finite_vector(
            packet.get("sensor_position_m", [0, 0, 0]), "sensor_position_m"),
        "tracks": tracks,
    }


def encode_signed_packet(packet: dict, shared_secret: str) -> bytes:
    """Encode a packet in an authenticated UDP envelope."""
    if not shared_secret:
        raise ValueError("shared_secret must not be empty")
    normalized = validate_packet(packet)
    payload = json.dumps(
        normalized, separators=(",", ":"), sort_keys=True).encode("utf-8")
    signature = hmac.new(
        shared_secret.encode("utf-8"), payload, hashlib.sha256).hexdigest()
    envelope = json.dumps({
        "payload": payload.decode("utf-8"),
        "hmac_sha256": signature,
    }, separators=(",", ":")).encode("utf-8")
    if len(envelope) > MAX_DATAGRAM_BYTES:
        raise ValueError("track packet is too large for one UDP datagram")
    return envelope


def decode_signed_packet(data: bytes, shared_secret: str) -> dict:
    """Authenticate, decode, and validate one UDP envelope."""
    if not shared_secret:
        raise ValueError("shared_secret must not be empty")
    if len(data) > MAX_DATAGRAM_BYTES:
        raise ValueError("track packet exceeds maximum datagram size")
    envelope = json.loads(data.decode("utf-8"))
    payload = envelope["payload"].encode("utf-8")
    expected = hmac.new(
        shared_secret.encode("utf-8"), payload, hashlib.sha256).hexdigest()
    if not hmac.compare_digest(expected, envelope.get("hmac_sha256", "")):
        raise ValueError("track packet authentication failed")
    return validate_packet(json.loads(payload))


def transform_track(track: dict, transform: np.ndarray) -> dict:
    """Transform position, velocity, and axis-aligned size to a shared frame."""
    rotation = transform[:3, :3]
    position = rotation @ np.asarray(track["position_m"]) + transform[:3, 3]
    velocity = rotation @ np.asarray(track["velocity_m_s"])
    # Rotating an axis-aligned box produces another box whose shared-frame
    # extents are |R| @ size. This is only a rough shape cue, not an oriented
    # bounding box.
    size = np.abs(rotation) @ np.asarray(track["size_m"])
    return {
        **track,
        "position_m": position.tolist(),
        "velocity_m_s": velocity.tolist(),
        "size_m": size.tolist(),
    }


class SharedTrackKnowledgeBase:
    """Fuse the latest two-source snapshots after repeated cross-dog matches."""

    def __init__(
        self,
        source_transforms: dict[str, np.ndarray],
        max_position_distance: float = 2.0,
        velocity_weight: float = 0.35,
        min_match_observations: int = 2,
        max_time_delta: float = 1.5,
        association_timeout: float = 5.0,
        snapshot_timeout: float = 2.5,
    ):
        if len(source_transforms) != 2:
            raise ValueError("the proof of concept currently requires two sources")
        if max_position_distance <= 0 or velocity_weight < 0:
            raise ValueError("matching distances/weights are invalid")
        if min_match_observations < 1:
            raise ValueError("min_match_observations must be at least one")
        self.transforms = {
            source: np.asarray(transform, dtype=float).reshape(4, 4)
            for source, transform in source_transforms.items()
        }
        self.max_position_distance = max_position_distance
        self.velocity_weight = velocity_weight
        self.min_match_observations = min_match_observations
        self.max_time_delta = max_time_delta
        self.association_timeout = association_timeout
        self.snapshot_timeout = snapshot_timeout
        self.latest = {}
        self._evidence = {}
        self._associations = {}
        self._last_matched_sequences = {}
        self._retired_sessions = {
            source: set() for source in self.transforms
        }

    def update(self, packet: dict) -> dict:
        packet = validate_packet(packet)
        source = packet["source_id"]
        if source not in self.transforms:
            raise ValueError(f"no shared-frame transform configured for {source!r}")
        previous = self.latest.get(source)
        if previous:
            if packet["session_id"] != previous["session_id"]:
                # Once a source has moved to a new process session, delayed
                # UDP packets from the old one must not switch it back.
                if packet["session_id"] in self._retired_sessions[source]:
                    newest_stamp = max(
                        item["stamp_s"] for item in self.latest.values())
                    return self._build_snapshot(newest_stamp)
                self._retired_sessions[source].add(previous["session_id"])
                self._forget_source(source)
            elif packet["sequence"] <= previous["sequence"]:
                newest_stamp = max(
                    item["stamp_s"] for item in self.latest.values())
                return self._build_snapshot(newest_stamp)
            elif packet["stamp_s"] < previous["stamp_s"]:
                self._forget_source(source)
        self.latest[source] = packet

        sources = sorted(self.transforms)
        if all(item in self.latest for item in sources):
            packet_a, packet_b = (self.latest[item] for item in sources)
            # Count evidence only after both sources have supplied a new
            # snapshot. ROS callbacks arrive independently; counting on each
            # callback would turn one physical observation pair into two
            # votes (new A + old B, then new A + new B).
            both_sources_advanced = all(
                self._last_matched_sequences.get(packet["source_id"])
                != packet["sequence"]
                for packet in (packet_a, packet_b)
            )
            if (
                both_sources_advanced
                and abs(packet_a["stamp_s"] - packet_b["stamp_s"])
                <= self.max_time_delta
            ):
                self._update_matches(packet_a, packet_b)
                self._last_matched_sequences = {
                    packet["source_id"]: packet["sequence"]
                    for packet in (packet_a, packet_b)
                }

        newest_stamp = max(item["stamp_s"] for item in self.latest.values())
        self._expire_associations(newest_stamp)
        return self._build_snapshot(newest_stamp)

    def _forget_source(self, source):
        self._evidence = {
            pair: value for pair, value in self._evidence.items()
            if pair[0][0] != source and pair[1][0] != source
        }
        self._associations = {
            key: value for key, value in self._associations.items()
            if key[0] != source and value[0] != source
        }
        self._last_matched_sequences.pop(source, None)

    def _shared_tracks(self, packet):
        transform = self.transforms[packet["source_id"]]
        return [transform_track(track, transform) for track in packet["tracks"]]

    def _update_matches(self, packet_a, packet_b):
        tracks_a = self._shared_tracks(packet_a)
        tracks_b = self._shared_tracks(packet_b)
        if not tracks_a or not tracks_b:
            return
        positions_a = np.array([track["position_m"] for track in tracks_a])
        positions_b = np.array([track["position_m"] for track in tracks_b])
        velocities_a = np.array([track["velocity_m_s"] for track in tracks_a])
        velocities_b = np.array([track["velocity_m_s"] for track in tracks_b])
        position_cost = np.linalg.norm(
            positions_a[:, None, :] - positions_b[None, :, :], axis=2)
        velocity_cost = np.linalg.norm(
            velocities_a[:, None, :] - velocities_b[None, :, :], axis=2)
        total_cost = position_cost + self.velocity_weight * velocity_cost
        rows, columns = linear_sum_assignment(total_cost)
        stamp = max(packet_a["stamp_s"], packet_b["stamp_s"])

        for row, column in zip(rows, columns):
            if position_cost[row, column] > self.max_position_distance:
                continue
            key_a = (packet_a["source_id"], tracks_a[row]["local_id"])
            key_b = (packet_b["source_id"], tracks_b[column]["local_id"])
            pair = tuple(sorted((key_a, key_b)))
            evidence = self._evidence.setdefault(
                pair, {"count": 0, "last_seen": stamp})
            evidence["count"] += 1
            evidence["last_seen"] = stamp
            if evidence["count"] >= self.min_match_observations:
                if (
                    key_a not in self._associations
                    and key_b not in self._associations
                ) or self._associations.get(key_a) == key_b:
                    self._associations[key_a] = key_b
                    self._associations[key_b] = key_a

    def _expire_associations(self, stamp):
        stale_pairs = [
            pair for pair, evidence in self._evidence.items()
            if stamp - evidence["last_seen"] > self.association_timeout
        ]
        for pair in stale_pairs:
            del self._evidence[pair]
            first, second = pair
            if self._associations.get(first) == second:
                del self._associations[first]
                del self._associations[second]

    def _global_id(self, key):
        other = self._associations.get(key)
        members = sorted((key, other)) if other else [key]
        return "+".join(f"{source}:{local_id}" for source, local_id in members)

    def _build_snapshot(self, newest_stamp):
        groups = {}
        frame_id = "shared"
        for source, packet in self.latest.items():
            if newest_stamp - packet["stamp_s"] > self.snapshot_timeout:
                continue
            for track in self._shared_tracks(packet):
                key = (source, track["local_id"])
                groups.setdefault(self._global_id(key), []).append(
                    (key, track, packet["stamp_s"]))

        fused_tracks = []
        for global_id, observations in sorted(groups.items()):
            positions = np.array([
                track["position_m"] for _, track, _ in observations])
            velocities = np.array([
                track["velocity_m_s"] for _, track, _ in observations])
            sizes = np.array([
                track["size_m"] for _, track, _ in observations])
            # A temporarily one-sided snapshot should retain the confidence
            # of its established association even while the peer has stopped
            # reporting that object. Looking only at the currently-present
            # observations would incorrectly display zero evidence.
            first_key = observations[0][0]
            associated_key = self._associations.get(first_key)
            pair = tuple(sorted((first_key, associated_key))) if associated_key else ()
            evidence = self._evidence.get(pair, {"count": 0})
            fused_tracks.append({
                "global_id": global_id,
                "contributors": {
                    source: local_id
                    for (source, local_id), _, _ in observations
                },
                "observation_stamps_s": {
                    source: stamp
                    for (source, _), _, stamp in observations
                },
                "position_m": positions.mean(axis=0).tolist(),
                "velocity_m_s": velocities.mean(axis=0).tolist(),
                "size_m": sizes.max(axis=0).tolist(),
                "n_points": sum(
                    track["n_points"] for _, track, _ in observations),
                "match_observations": evidence["count"],
            })
        return {
            "schema_version": SCHEMA_VERSION,
            "stamp_s": newest_stamp,
            "frame_id": frame_id,
            "sources": sorted(self.latest),
            "tracks": fused_tracks,
        }
