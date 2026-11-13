#!/usr/bin/env bash
set -euo pipefail
exec /ros_entrypoint.sh python3 /opt/greenhouse/tools/teleop.py "$@"
