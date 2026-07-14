#!/bin/bash
# entrypoint.sh -- source the workspace, then dispatch to whichever piece
# of the stack was asked for. Only the "driver" command touches the LiDAR
# config -- bag replay, FastLIO, and a plain shell don't talk to a physical
# sensor at all, so they shouldn't be blocked on a LIVOX_LIDAR_IP that has
# nothing to do with them.
#
# Deliberately not using `set -u`: ROS's own setup.bash scripts reference
# a handful of environment variables (e.g. AMENT_TRACE_SETUP_FILES) without
# defaulting them first, which trips nounset immediately on `source`. This
# is a known ROS2 gotcha, not something worth patching around upstream.
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/ros2_ws/install/setup.bash

render_livox_config() {
    # LIVOX_LIDAR_IP is required here specifically -- each Mid-360's IP is
    # tied to its serial number and must match whichever physical unit is
    # plugged into this dog (see DOCS.md's hardware reference table).
    # Getting this wrong doesn't fail loudly: livox_ros_driver2 just reports
    # "Storage point data failed" and never publishes any points, which
    # looks like a much harder-to-diagnose problem than a plain "wrong
    # config" would.
    : "${LIVOX_HOST_IP:=192.168.1.50}"
    : "${LIVOX_LIDAR_IP:?LIVOX_LIDAR_IP must be set to the Mid-360 IP for this dog (e.g. 192.168.1.137 for Dog 1, 192.168.1.120 for Dog 2)}"

    local config_path=/opt/ros2_ws/src/livox_ros_driver2/config/MID360_config.json
    LIVOX_HOST_IP="$LIVOX_HOST_IP" LIVOX_LIDAR_IP="$LIVOX_LIDAR_IP" \
        envsubst '${LIVOX_HOST_IP} ${LIVOX_LIDAR_IP}' \
        < /opt/MID360_config.json.template \
        > "$config_path"
    echo "Rendered $config_path (host=$LIVOX_HOST_IP, lidar=$LIVOX_LIDAR_IP)"
}

case "${1:-driver}" in
    driver)
        render_livox_config
        exec ros2 launch livox_ros_driver2 msg_MID360_launch.py
        ;;
    fastlio)
        exec ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
        ;;
    bash|shell)
        shift
        exec bash "$@"
        ;;
    *)
        # Anything else -- pass straight through so `docker run <image>
        # ros2 bag play ...` / `ros2 topic list` and similar still work
        # without a special case.
        exec "$@"
        ;;
esac
