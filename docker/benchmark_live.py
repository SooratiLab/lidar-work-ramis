#!/usr/bin/env python3
"""Collect a repeatable live-pipeline benchmark on a Go2 Jetson.

The hardware Docker profile must already be running.  This script deliberately
runs on the host: Docker and tegrastats already expose the useful whole-system
measurements there, without adding tools or overhead to the runtime image.
"""

import argparse
import csv
import datetime as dt
import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path


CONTAINERS = (
    "go2-lidar-driver",
    "go2-fastlio",
    "go2-online-perception",
    "go2-cluster-response",
    "go2-stop-actuation",
)
TOPICS = (
    "/livox/lidar",
    "/livox/imu",
    "/cloud_registered",
    "/Odometry",
    "/online_perception/tracks",
    "/online_perception/stop_requested",
)
FRAME_RE = re.compile(r"dt=([0-9.]+)s, processed in ([0-9.]+)ms")


def run(command, *, timeout=None):
    return subprocess.run(
        command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        timeout=timeout, check=False,
    )


def percentile(values, fraction):
    """Nearest-rank percentile, sufficient for an operational summary."""
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(len(ordered) * fraction + 0.999999) - 1))
    return ordered[index]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=300,
                        help="measurement duration in seconds (default: 300)")
    parser.add_argument("--output", type=Path,
                        help="output directory (default: benchmark-<timestamp>)")
    return parser.parse_args()


def require_live_stack():
    missing = []
    for name in CONTAINERS:
        result = run(["docker", "inspect", "-f", "{{.State.Running}}", name])
        if result.returncode != 0 or result.stdout.strip() != "true":
            missing.append(name)
    if missing:
        print("The hardware profile is not ready; missing/running=false: " + ", ".join(missing),
              file=sys.stderr)
        print("Start it with: docker compose --profile hardware up -d", file=sys.stderr)
        raise SystemExit(2)


def capture_platform(output_dir):
    commands = {
        "date.txt": ["date", "--iso-8601=seconds"],
        "uname.txt": ["uname", "-a"],
        "docker-version.txt": ["docker", "version"],
        "docker-images.txt": ["docker", "images", "--digests", "go2-lidar-humble"],
        "container-state.json": ["docker", "inspect", *CONTAINERS],
        "jetson-release.txt": ["bash", "-lc", "cat /etc/nv_tegra_release 2>/dev/null; dpkg-query -W nvidia-jetpack 2>/dev/null"],
        "power-mode.txt": ["nvpmodel", "-q"],
    }
    for filename, command in commands.items():
        if shutil.which(command[0]) is None:
            continue
        result = run(command, timeout=30)
        (output_dir / filename).write_text(result.stdout, encoding="utf-8")


def start_topic_monitors(duration, output_dir):
    processes = []
    for topic in TOPICS:
        safe_name = topic.strip("/").replace("/", "-")
        handle = (output_dir / f"topic-hz-{safe_name}.txt").open("w", encoding="utf-8")
        command = [
            "docker", "exec", "go2-fastlio", "bash", "-lc",
            f"timeout {duration}s ros2 topic hz {topic}",
        ]
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, text=True)
        processes.append((process, handle))
    return processes


def start_tegrastats(output_dir):
    executable = shutil.which("tegrastats")
    if executable is None:
        return None
    handle = (output_dir / "tegrastats.txt").open("w", encoding="utf-8")
    process = subprocess.Popen([executable, "--interval", "1000"], stdout=handle,
                               stderr=subprocess.STDOUT, text=True)
    return process, handle


def sample_container_stats(duration, output_dir):
    deadline = time.monotonic() + duration
    path = output_dir / "docker-stats.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["timestamp", "container", "cpu_percent", "memory_usage", "memory_percent"])
        while time.monotonic() < deadline:
            result = run([
                "docker", "stats", "--no-stream", "--format",
                "{{.Name}}|{{.CPUPerc}}|{{.MemUsage}}|{{.MemPerc}}", *CONTAINERS,
            ], timeout=15)
            timestamp = dt.datetime.now(dt.timezone.utc).isoformat()
            for line in result.stdout.splitlines():
                fields = line.split("|", 3)
                if len(fields) == 4:
                    writer.writerow([timestamp, *fields])
            stream.flush()
            time.sleep(1)


def stop_process(item):
    if item is None:
        return
    process, handle = item
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    handle.close()


def summarize_perception(started_at, output_dir):
    since = started_at.astimezone(dt.timezone.utc).isoformat()
    result = run(["docker", "logs", "--since", since, "go2-online-perception"], timeout=30)
    (output_dir / "perception.log").write_text(result.stdout, encoding="utf-8")

    processing_ms = []
    budget_fraction = []
    for match in FRAME_RE.finditer(result.stdout):
        frame_dt = float(match.group(1))
        elapsed_ms = float(match.group(2))
        processing_ms.append(elapsed_ms)
        if frame_dt > 0:
            budget_fraction.append(elapsed_ms / (frame_dt * 1000.0))

    summary = {
        "measured_frames": len(processing_ms),
        "processing_ms": {
            "mean": sum(processing_ms) / len(processing_ms) if processing_ms else None,
            "p50": percentile(processing_ms, 0.50),
            "p95": percentile(processing_ms, 0.95),
            "p99": percentile(processing_ms, 0.99),
            "max": max(processing_ms) if processing_ms else None,
        },
        "realtime_budget_percent": {
            "p95": percentile(budget_fraction, 0.95) * 100 if budget_fraction else None,
            "max": max(budget_fraction) * 100 if budget_fraction else None,
        },
        "warning_count": result.stdout.count("[WARN]"),
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def capture_response_logs(started_at, output_dir, summary):
    since = started_at.astimezone(dt.timezone.utc).isoformat()
    components = {
        "response.log": "go2-cluster-response",
        "actuation.log": "go2-stop-actuation",
    }
    logs = {}
    for filename, container in components.items():
        result = run(["docker", "logs", "--since", since, container], timeout=30)
        (output_dir / filename).write_text(result.stdout, encoding="utf-8")
        logs[filename] = result.stdout

    summary["response"] = {
        "stop_transitions_logged": logs["response.log"].count("STOP REQUESTED"),
        "clear_transitions_logged": logs["response.log"].count(
            "stop request cleared"),
        "stale_state_entries_logged": logs["response.log"].count(
            "state=stale_"),
    }
    summary["actuation"] = {
        "enabled": "enabled=True" in logs["actuation.log"],
        "dry_run_stop_actions": logs["actuation.log"].count(
            "would_send_stop_move"),
        "sent_stop_actions": logs["actuation.log"].count("sent_stop_move"),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")


def main():
    args = parse_args()
    if args.duration < 10:
        raise SystemExit("--duration must be at least 10 seconds")
    if shutil.which("docker") is None:
        raise SystemExit("docker is not installed or not on PATH")

    require_live_stack()
    started_at = dt.datetime.now().astimezone()
    output_dir = args.output or Path(f"benchmark-{started_at.strftime('%Y%m%d-%H%M%S')}")
    output_dir.mkdir(parents=True, exist_ok=False)
    capture_platform(output_dir)

    print(f"Collecting {args.duration}s benchmark into {output_dir} ...")
    topic_processes = start_topic_monitors(args.duration, output_dir)
    tegrastats_process = start_tegrastats(output_dir)
    try:
        sample_container_stats(args.duration, output_dir)
    except KeyboardInterrupt:
        print("Interrupted; keeping the partial benchmark.")
    finally:
        for item in topic_processes:
            stop_process(item)
        stop_process(tegrastats_process)

    summary = summarize_perception(started_at, output_dir)
    capture_response_logs(started_at, output_dir, summary)
    print(json.dumps(summary, indent=2))
    if not summary["measured_frames"]:
        print("No processed frames were found; inspect perception.log and topic-hz files.",
              file=sys.stderr)
        return 1
    print(f"Benchmark complete: {output_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
