#!/usr/bin/env bash
set -euo pipefail

root=/opt/greenhouse
original_args=("$@")
setup_path="${SETUP_PATH:-/etc/clearpath}"
mode=run
headless=false
rviz=false
viz=false
gui_test=false
extra=()
while (($#)); do
  case "$1" in
    --smoke-test) mode=smoke ;;
    --generate-only) mode=generate ;;
    headless:=true|headless:=false) headless="${1#headless:=}" ;;
    rviz:=true|rviz:=false) rviz="${1#rviz:=}" ;;
    viz:=true|viz:=false) viz="${1#viz:=}" ;;
    gui_test:=true|gui_test:=false|gui_test:=remaining) gui_test="${1#gui_test:=}" ;;
    --show-args|--debug) extra+=("$1") ;;
    *) printf 'Unsupported launch option: %s\nUse headless:=true, rviz:=true, viz:=true, --show-args or --debug.\n' "$1" >&2; exit 2 ;;
  esac
  shift
done
if [[ "$mode" == smoke && "${GREENHOUSE_SMOKE_GUARD:-0}" != 1 ]]; then
  exec timeout --signal=INT --kill-after=15s 240s \
    env GREENHOUSE_SMOKE_GUARD=1 bash "$0" "${original_args[@]}"
fi
if [[ "$headless" == true && "$rviz" == true ]]; then
  echo "Headless mode cannot launch RViz." >&2
  exit 2
fi
if [[ ! -d "$setup_path" || ! -w "$setup_path" ]]; then
  echo "SETUP_PATH must be a writable mounted setup directory: $setup_path" >&2
  exit 2
fi
mkdir -p "$setup_path/greenhouse"
export GREENHOUSE_STATE="$setup_path/greenhouse"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export QT_X11_NO_MITSHM=1
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/greenhouse-runtime-$(id -u)}"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

if [[ "$mode" == generate ]]; then
  exec env PYTHONPATH="$root/python:$root${PYTHONPATH:+:$PYTHONPATH}" \
    python3 -m greenhouse.generate --config "$GREENHOUSE_CONFIG" \
    --output "$GREENHOUSE_STATE/generated"
fi
# Objects of an earlier run in this setup dir are gone; a scenario starts a fresh state file.
rm -f "$GREENHOUSE_STATE/scenario-state.json"
if [[ -n "${GREENHOUSE_SCENARIO:-}" ]]; then
  python3 -m greenhouse.scenario prepare --scenario "$GREENHOUSE_SCENARIO" --config "$GREENHOUSE_CONFIG" \
    --actuator-config "${ACTUATOR_CONFIG:-$root/config/actuator.json}" --output "$GREENHOUSE_STATE"
  export GREENHOUSE_CONFIG="$GREENHOUSE_STATE/scenario-greenhouse.json"
  export ACTUATOR_CONFIG="$GREENHOUSE_STATE/scenario-actuator.json"
fi
python3 -m greenhouse.runtime prepare --headless "$headless"
export GREENHOUSE_ASSETS
GREENHOUSE_ASSETS="$(python3 -m greenhouse.runtime assets)"
if [[ "$GREENHOUSE_ASSETS" != "$root/generated" ]]; then
  env PYTHONPATH="$root/python:$root${PYTHONPATH:+:$PYTHONPATH}" \
    python3 -m greenhouse.generate --config "$GREENHOUSE_CONFIG" --output "$GREENHOUSE_ASSETS"
fi

# The same four executables used by robot_spawn.launch.py, with checked exits.
ros2 run clearpath_generator_common generate_description -s "$setup_path"
ros2 run clearpath_generator_common generate_semantic_description -s "$setup_path"
ros2 run clearpath_generator_gz generate_launch -s "$setup_path"
ros2 run clearpath_generator_gz generate_param -s "$setup_path"
namespace="$(python3 -c 'from greenhouse.runtime import read_state; print(read_state()["namespace"])')"
python3 -m greenhouse.mount --config "${ACTUATOR_CONFIG:-$root/config/actuator.json}" \
  --output "$GREENHOUSE_STATE/cad"
python3 -m greenhouse.actuator --config "${ACTUATOR_CONFIG:-$root/config/actuator.json}" \
  --setup "$setup_path" --namespace "$namespace"
python3 -m greenhouse.runtime check-generated
python3 "$root/tools/prepare_navigation.py"
python3 "$root/tools/prepare_platform.py"

launch=(ros2 launch "$root/launch/greenhouse.launch.py"
        "setup_path:=$setup_path" "assets_dir:=$GREENHOUSE_ASSETS"
        "headless:=$headless" "rviz:=$rviz" "viz:=$viz" "gui_test:=$gui_test" "${extra[@]}")
if [[ "$mode" == run ]]; then
  # A --script from run.sh waits for readiness on its own, runs once, and leaves the sim running.
  # Backgrounded before exec so ros2 launch keeps this PID and its Ctrl+C/stop handling.
  if [[ -n "${GREENHOUSE_SCENARIO:-}" ]]; then
    python3 "$root/tools/scenario.py" run "$GREENHOUSE_SCENARIO" &
  fi
  if [[ -n "${GREENHOUSE_SCRIPT:-}" ]]; then
    (cd -- "$(dirname -- "$GREENHOUSE_SCRIPT")" &&
      exec python3 "$root/self_driving/run_script.py" "$GREENHOUSE_SCRIPT") &
  fi
  exec "${launch[@]}"
fi

log="$GREENHOUSE_STATE/smoke-launch.log"
"${launch[@]}" >"$log" 2>&1 &
launch_pid=$!
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid"
    for ((i=0; i<100; i++)); do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$launch_pid" 2>/dev/null; then kill -TERM "$launch_pid"; fi
    for ((i=0; i<30; i++)); do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$launch_pid" 2>/dev/null; then kill -KILL "$launch_pid"; fi
    wait "$launch_pid" || true
  fi
  cat "$log"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 "$root/tools/smoke_simulation.py" --timeout 180 \
  --launch-pid "$launch_pid" --output "$GREENHOUSE_STATE/smoke-result.json"
