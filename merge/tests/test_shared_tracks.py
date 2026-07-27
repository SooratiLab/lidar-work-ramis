import json

import numpy as np
import pytest

from shared_tracks import (
    SharedTrackKnowledgeBase,
    build_transform,
    decode_signed_packet,
    encode_signed_packet,
    transform_track,
    validate_packet,
)


def packet(source, sequence, stamp, tracks, session="test-session"):
    return {
        "schema_version": 1,
        "source_id": source,
        "session_id": session,
        "sequence": sequence,
        "stamp_s": stamp,
        "frame_id": f"{source}/map",
        "sensor_position_m": [0, 0, 0],
        "tracks": tracks,
    }


def track(local_id, position, velocity=(0, 0, 0), size=(1, 1, 1), points=10):
    return {
        "local_id": local_id,
        "position_m": position,
        "velocity_m_s": velocity,
        "size_m": size,
        "n_points": points,
    }


def knowledge_base(**kwargs):
    return SharedTrackKnowledgeBase(
        {"dog1": np.eye(4), "dog2": np.eye(4)}, **kwargs)


def test_packet_signing_round_trip_is_canonical_and_authenticated():
    original = packet("dog1", 7, 12.5, [track(3, [1, 2, 3])])

    encoded = encode_signed_packet(original, "test secret")
    decoded = decode_signed_packet(encoded, "test secret")

    assert decoded == validate_packet(original)
    with pytest.raises(ValueError, match="authentication failed"):
        decode_signed_packet(encoded, "wrong secret")

    envelope = json.loads(encoded)
    envelope["payload"] = envelope["payload"].replace('"sequence":7', '"sequence":8')
    with pytest.raises(ValueError, match="authentication failed"):
        decode_signed_packet(json.dumps(envelope).encode(), "test secret")


@pytest.mark.parametrize(
    "broken",
    [
        packet("dog1", 0, 1.0, [track(1, [0, 0, 0]), track(1, [1, 0, 0])]),
        packet("dog1", 0, float("nan"), []),
        packet("dog1", 0, 1.0, [track(1, [float("inf"), 0, 0])]),
        packet("dog1", 0, 1.0, [track(1, [0, 0, 0], size=[-1, 1, 1])]),
    ],
)
def test_packet_validation_rejects_ambiguous_or_nonfinite_data(broken):
    with pytest.raises(ValueError):
        validate_packet(broken)


def test_transform_rotates_position_velocity_and_axis_aligned_size():
    transformed = transform_track(
        track(1, [1, 0, 0], velocity=[1, 0, 0], size=[2, 1, 1]),
        build_transform("10,20,1,90"),
    )

    assert transformed["position_m"] == pytest.approx([10, 21, 1])
    assert transformed["velocity_m_s"] == pytest.approx([0, 1, 0])
    assert transformed["size_m"] == pytest.approx([1, 2, 1])


def test_single_dog_tracks_are_kept_as_complementary_coverage():
    snapshot = knowledge_base().update(
        packet("dog1", 0, 1.0, [track(4, [1, 2, 0])]))

    assert snapshot["sources"] == ["dog1"]
    assert snapshot["tracks"][0]["global_id"] == "dog1:4"
    assert snapshot["tracks"][0]["contributors"] == {"dog1": 4}


def test_match_requires_repeated_time_aligned_observations_then_fuses():
    kb = knowledge_base(min_match_observations=2)
    kb.update(packet("dog1", 0, 1.0, [track(1, [0, 0, 0], points=10)]))
    first = kb.update(
        packet("dog2", 0, 1.1, [track(8, [0.2, 0, 0], points=12)]))
    assert {item["global_id"] for item in first["tracks"]} == {
        "dog1:1", "dog2:8"}

    kb.update(packet("dog1", 1, 2.0, [track(1, [1, 0, 0], points=11)]))
    fused = kb.update(
        packet("dog2", 1, 2.1, [track(8, [1.2, 0, 0], points=13)]))

    assert len(fused["tracks"]) == 1
    shared = fused["tracks"][0]
    assert shared["global_id"] == "dog1:1+dog2:8"
    assert shared["contributors"] == {"dog1": 1, "dog2": 8}
    assert shared["observation_stamps_s"] == {"dog1": 2.0, "dog2": 2.1}
    assert shared["position_m"] == pytest.approx([1.1, 0, 0])
    assert shared["n_points"] == 24
    assert shared["match_observations"] == 2


def test_out_of_sync_snapshots_are_never_associated():
    kb = knowledge_base(max_time_delta=0.2, min_match_observations=1)
    kb.update(packet("dog1", 0, 1.0, [track(1, [0, 0, 0])]))
    snapshot = kb.update(packet("dog2", 0, 2.0, [track(8, [0, 0, 0])]))

    assert len(snapshot["tracks"]) == 2


def test_velocity_resolves_nearby_crossing_tracks():
    kb = knowledge_base(min_match_observations=1, velocity_weight=1.0)
    kb.update(packet("dog1", 0, 1.0, [
        track(1, [-0.1, 0, 0], velocity=[1, 0, 0]),
        track(2, [0.1, 0, 0], velocity=[-1, 0, 0]),
    ]))
    snapshot = kb.update(packet("dog2", 0, 1.0, [
        track(8, [0.1, 0, 0], velocity=[1, 0, 0]),
        track(9, [-0.1, 0, 0], velocity=[-1, 0, 0]),
    ]))

    assert {item["global_id"] for item in snapshot["tracks"]} == {
        "dog1:1+dog2:8", "dog1:2+dog2:9"}


def test_expired_association_splits_tracks_again():
    kb = knowledge_base(
        min_match_observations=1,
        association_timeout=1.0,
        snapshot_timeout=10.0,
    )
    kb.update(packet("dog1", 0, 1.0, [track(1, [0, 0, 0])]))
    matched = kb.update(packet("dog2", 0, 1.0, [track(8, [0, 0, 0])]))
    assert len(matched["tracks"]) == 1

    split = kb.update(packet("dog1", 1, 3.0, [track(1, [5, 0, 0])]))
    assert {item["global_id"] for item in split["tracks"]} == {
        "dog1:1", "dog2:8"}


def test_clock_regression_forgets_old_source_association_evidence():
    kb = knowledge_base(min_match_observations=2, snapshot_timeout=10.0)
    kb.update(packet("dog1", 10, 10.0, [track(1, [0, 0, 0])]))
    kb.update(packet("dog2", 10, 10.0, [track(8, [0, 0, 0])]))
    kb.update(packet("dog1", 11, 11.0, [track(1, [0, 0, 0])]))
    matched = kb.update(packet("dog2", 11, 11.0, [track(8, [0, 0, 0])]))
    assert len(matched["tracks"]) == 1

    # A replay loop or restarted clock must not inherit confidence from the
    # previous timeline.
    regressed = kb.update(packet("dog1", 12, 1.0, [track(1, [0, 0, 0])]))
    assert {item["global_id"] for item in regressed["tracks"]} == {
        "dog1:1", "dog2:8"}


def test_new_session_resets_ids_and_late_old_session_is_ignored():
    kb = knowledge_base(min_match_observations=1, snapshot_timeout=10.0)
    kb.update(packet("dog1", 10, 10.0, [track(1, [0, 0, 0])], "old"))
    kb.update(packet("dog2", 10, 10.0, [track(8, [0, 0, 0])], "peer"))

    restarted = kb.update(
        packet("dog1", 1, 11.0, [track(1, [5, 0, 0])], "new"))
    assert {item["global_id"] for item in restarted["tracks"]} == {
        "dog1:1", "dog2:8"}

    delayed = kb.update(
        packet("dog1", 11, 10.5, [track(1, [0, 0, 0])], "old"))
    dog1_track = next(
        item for item in delayed["tracks"] if item["global_id"] == "dog1:1")
    assert dog1_track["position_m"] == pytest.approx([5, 0, 0])
