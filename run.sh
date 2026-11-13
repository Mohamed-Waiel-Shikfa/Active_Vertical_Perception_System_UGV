#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
image=jackal-greenhouse:jazzy
name=jackal_greenhouse
setup_dir=
config="$script_dir/config/greenhouse.json"
actuator_config="$script_dir/config/actuator.json"
headless=false
software=false
build=true
build_only=false
dry_run=false
mode=run
rviz=false
viz=false
gui_test=false
allow_stale=false
script=
scenario=
extra=()

usage() {
  cat <<'USAGE'
Usage: bash run.sh [options] [-- --show-args|--debug]

Runs one Clearpath Jackal in the generated greenhouse, all inside one Docker container.
With no options: build the image if the sources changed (Docker cache makes an unchanged
build quick), then open the Gazebo window with the drive panel.

What to do (pick at most one; default = interactive simulation)
  --smoke-test           Headless start, short safe drive, then exit 0 (pass) or non-zero.
                         Bounded to 4 min. Implies --headless.
  --generate-only        Write the world assets to SETUP_DIR/greenhouse/generated and exit.
                         No ROS, no Gazebo. Implies --headless.
  --build-only           Build the image and exit. Starts nothing.
  --source-fingerprint   Print the hash of the sources an image is built from, and exit.

How it is displayed
  --headless             No window. Gazebo still renders the cameras off-screen, on the GPU
                         when /dev/dri exists. For SSH, CI or scripted runs. Not with --rviz
                         or --gui-test.
  --software-rendering   Render on the CPU (Mesa llvmpipe) instead of the GPU. Works with or
                         without a window, but the simulation runs 2-4x slower than real
                         time (measured RTF 0.5 headless, 0.23 with the window). Only use it
                         when GPU rendering fails.
  --rviz                 Also open RViz next to Gazebo.
  --viz                  Visualisation hub: start foxglove_bridge on ws://127.0.0.1:8767 (this
                         computer only) plus odometry-error, greyscale camera and ROS graph
                         topics. Open it in Lichtblick and import config/foxglove-layout.json;
                         its "System graph" tab needs bash viz/subsystem-graph/install.sh once.
                         Headless or with the window; one --viz simulation at a time.

Which image
  --no-build             Start the existing image without building. Refused if that image was
                         built from different sources than this folder (see
                         --source-fingerprint). Editing config/greenhouse.json or
                         config/actuator.json never needs a rebuild; the other config/
                         files are baked into the image and do.
  --allow-stale-image    With --no-build: start the mismatched image anyway, with a warning.
  --image TAG            Image tag to build/start (default: jackal-greenhouse:jazzy).

Inputs (relative DIR/FILE paths are resolved from this folder, except --script)
  --setup-dir DIR        Writable Clearpath folder holding robot.yaml; run state goes to
                         DIR/greenhouse/. Default: this folder if ./robot.yaml exists, else
                         runtime/clearpath (seeded from config/robot.example.yaml once).
  --config FILE          Greenhouse JSON. Keys it omits come from config/greenhouse.json.
  --actuator-config FILE Lift and gimbal JSON. Keys it omits come from config/actuator.json.
  --script FILE          Python script using self_driving_api (see self_driving/). Runs once
                         the simulation is ready; the simulation stays open afterwards. FILE
                         is relative to the current directory (else to this folder); its
                         folder is mounted read-write and is the script's working directory.
                         Only with the default interactive mode (headless is fine).
  --scenario FILE        Scenario JSON (see config/scenario.example.json): world seed, greenhouse
                         and actuator overrides (merged over --config/--actuator-config), robot
                         start, perturbation objects and timed events. Mounted read-only; FILE is
                         relative to the current directory (else to this folder). Only with the
                         default interactive mode (headless is fine).
  --name NAME            Container name (default: jackal_greenhouse). Two simulations at once
                         need different names and different --setup-dir folders.

Checks and debugging
  --dry-run              Print the docker commands that would run. Runs and writes nothing.
  --gui-test             Self-test of the GUI: drives with synthetic keys and checks the panel.
                         Needs the window focused: do not touch the desktop while it runs.
  --gui-test-remaining   Same, skipping the drive section.
  -- --show-args         Passed to ros2 launch: list the launch arguments.
  -- --debug             Passed to ros2 launch: verbose launch logging.
  --help                 Show this help.

Typical use
  bash run.sh                                        first start, or after code changes
  bash run.sh --no-build                             restart quickly (config JSON edits only)
  bash run.sh --script self_driving/sample_use.py    run your scripted poses
  bash run.sh --headless --script my_test.py         scripted run without a window
  bash run.sh --headless --viz                       watch it in Foxglove at ws://127.0.0.1:8767
  bash run.sh --scenario config/scenario.example.json   seeded world with obstacles and events
  bash run.sh --smoke-test                           check the whole stack still starts
  bash run.sh --config my.json --name gh2 --setup-dir /tmp/gh2   second sim with another layout

No ROS, Python packages or graphics settings are installed or changed on the host.
USAGE
}
die() { printf 'greenhouse: %s\n' "$*" >&2; exit 2; }
# Hash of everything the image runs: code, launch, native sources and build-only config.
# Runtime-mounted JSON (greenhouse/actuator configs) and docs/tests are excluded, so editing
# them never forces a rebuild. Stored as an image label to catch a stale --no-build image.
source_fingerprint() {
  local paths=() path
  for path in Dockerfile requirements-generator.txt greenhouse third_party launch docker tools native \
              self_driving/self_driving_api.py self_driving/run_script.py \
              config/nav-packages.txt config/nav-packages.lock.json config/navigation.yaml \
              config/navigate.xml config/platform_overrides.yaml config/robot.example.yaml; do
    [[ -e "$script_dir/$path" ]] && paths+=("$path")
  done
  ((${#paths[@]})) || { printf 'none\n'; return; }
  (cd -- "$script_dir" &&
    find "${paths[@]}" -type f ! -path '*/__pycache__/*' ! -name '*.pyc' -print0 |
      LC_ALL=C sort -z | xargs -0 sha256sum | sha256sum | cut -c1-16)
}
value() { (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || die "$1 needs a value"; }
absolute() {
  local path="$1"
  [[ "$path" = /* ]] || path="$script_dir/$path"
  realpath -m -- "$path"
}
while (($#)); do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --headless) headless=true ;;
    --software-rendering) software=true ;;
    --smoke-test) [[ "$mode" == run ]] || die "choose only one run mode"; mode=smoke; headless=true ;;
    --generate-only) [[ "$mode" == run ]] || die "choose only one run mode"; mode=generate; headless=true ;;
    --setup-dir) value "$@"; setup_dir="$(absolute "$2")"; shift ;;
    --config) value "$@"; config="$(absolute "$2")"; shift ;;
    --actuator-config) value "$@"; actuator_config="$(absolute "$2")"; shift ;;
    --name) value "$@"; name="$2"; shift ;;
    --image) value "$@"; image="$2"; shift ;;
    --no-build) build=false ;;
    --allow-stale-image) allow_stale=true ;;
    --source-fingerprint) source_fingerprint; exit 0 ;;
    --build-only) build_only=true ;;
    --rviz) rviz=true ;;
    --viz) viz=true ;;
    --script) value "$@"; script="$2"; shift ;;
    --scenario) value "$@"; scenario="$2"; shift ;;
    --gui-test) gui_test=true ;;
    --gui-test-remaining) gui_test=remaining ;;
    --show-args|--debug) extra+=("$1") ;;
    --dry-run) dry_run=true ;;
    --)
      shift
      for argument in "$@"; do
        case "$argument" in
          --show-args|--debug) extra+=("$argument") ;;
          *) die "unsupported ROS launch option: $argument (world and spawn come from greenhouse.json)" ;;
        esac
      done
      break ;;
    *) die "unknown option: $1 (use --help)" ;;
  esac
  shift
done
[[ "$name" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]] || die "invalid container name: $name"
[[ "$image" =~ ^[a-z0-9][a-z0-9./:_-]*$ ]] || die "invalid local image tag: $image"
[[ "$headless" != true || "$rviz" != true ]] || die "--rviz cannot be combined with headless mode"
[[ "$headless" != true || "$gui_test" == false ]] || die "--gui-test requires the real GUI"
[[ "$build_only" != true || "$build" == true ]] || die "--build-only conflicts with --no-build"
[[ "$allow_stale" != true || "$build" == false ]] || die "--allow-stale-image only applies with --no-build"
[[ "$build_only" != true || "$mode" == run ]] || die "--build-only conflicts with the selected run mode"
[[ "$viz" != true || ("$mode" != generate && "$build_only" == false) ]] || die "--viz needs a simulation (not --generate-only or --build-only)"
[[ -f "$config" && -r "$config" ]] || die "greenhouse JSON is not readable: $config"
if [[ -n "$script" ]]; then
  [[ "$mode" == run && "$build_only" == false ]] || die "--script needs a normal run (not smoke/generate/build-only)"
  if [[ "$script" != /* && -e "$PWD/$script" ]]; then script="$PWD/$script"; fi
  script="$(absolute "$script")"
  [[ -f "$script" && -r "$script" && "$script" == *.py ]] || die "--script must be a readable .py file: $script"
fi
if [[ -n "$scenario" ]]; then
  [[ "$mode" == run && "$build_only" == false ]] || die "--scenario needs a normal run (not smoke/generate/build-only)"
  if [[ "$scenario" != /* && -e "$PWD/$scenario" ]]; then scenario="$PWD/$scenario"; fi
  scenario="$(absolute "$scenario")"
  [[ -f "$scenario" && -r "$scenario" && "$scenario" == *.json ]] || die "--scenario must be a readable .json file: $scenario"
fi
[[ -f "$actuator_config" && -r "$actuator_config" ]] || die "actuator JSON is not readable: $actuator_config"

default_setup=false
if [[ -z "$setup_dir" ]]; then
  if [[ -f "$script_dir/robot.yaml" ]]; then
    setup_dir="$script_dir"
  else
    setup_dir="$script_dir/runtime/clearpath"
    default_setup=true
  fi
fi
for path in "$setup_dir" "$config" "$actuator_config" "$script_dir" ${script:+"$script"} ${scenario:+"$scenario"}; do
  [[ "$path" != *","* && "$path" != *$'\n'* && "$path" != *$'\r'* ]] ||
    die "Docker bind paths cannot contain commas or line breaks: $path"
done
print_command() { printf '%q ' "$@"; printf '\n'; }
fingerprint="$(source_fingerprint)"
build_command=(docker build --build-arg "GREENHOUSE_SOURCE=$fingerprint" --tag "$image" "$script_dir")
if [[ "$dry_run" == true ]]; then
  [[ "$build" == false ]] || print_command "${build_command[@]}"
  [[ "$build_only" == false ]] || exit 0
else
  command -v docker >/dev/null || die "Docker CLI missing; install Docker Engine and add your user to the docker group"
  server_os="$(docker info --format '{{.OSType}}')" ||
    die "cannot reach the Docker daemon; start it (sudo systemctl start docker) and check that your user can run docker"
  [[ "$server_os" == linux ]] || die "Docker must be using Linux containers, not $server_os"
  if [[ "$build" == true ]]; then
    "${build_command[@]}"
  else
    docker image inspect "$image" >/dev/null ||
      die "image $image is missing; run once without --no-build"
    built_from="$(docker image inspect --format \
      '{{ with .Config.Labels }}{{ index . "org.jackal-greenhouse.source" }}{{ end }}' "$image")" ||
      die "cannot read the source label of image $image"
    if [[ "$built_from" != "$fingerprint" ]]; then
      [[ "$allow_stale" == true ]] ||
        die "image $image was built from different source (image: ${built_from:-unlabelled}, folder: $fingerprint). Its code would not match this folder's config. Rebuild by running without --no-build, or pass --allow-stale-image to start it anyway."
      printf 'greenhouse: warning: starting stale image %s (built from %s, folder is %s)\n' \
        "$image" "${built_from:-unlabelled}" "$fingerprint" >&2
    fi
  fi
  [[ "$build_only" == false ]] || exit 0
  if docker container inspect "$name" >/dev/null 2>&1; then
    die "container '$name' already exists. Choose --name NAME or inspect/remove that specific container yourself. Nothing was deleted."
  fi
  if [[ "$default_setup" == true ]]; then
    mkdir -p -- "$setup_dir"
    if [[ ! -e "$setup_dir/robot.yaml" ]]; then
      (umask 077; cp -n -- "$script_dir/config/robot.example.yaml" "$setup_dir/robot.yaml")
      printf 'Initialized example robot.yaml at %s\n' "$setup_dir"
    fi
  fi
  [[ -d "$setup_dir" && -w "$setup_dir" ]] || die "setup directory must exist and be writable: $setup_dir"
  [[ -f "$setup_dir/robot.yaml" && -r "$setup_dir/robot.yaml" ]] || die "missing readable $setup_dir/robot.yaml"
fi

args=(docker run --rm --init --stop-timeout 15 --name "$name"
      --label "org.jackal-greenhouse.role=simulation"
      --shm-size 512m --user "$(id -u):$(id -g)"
      --env HOME=/tmp/greenhouse-home
      --env SETUP_PATH=/etc/clearpath
      --env GREENHOUSE_CONFIG=/opt/greenhouse/config/selected.json
      --env ACTUATOR_CONFIG=/opt/greenhouse/config/selected-actuator.json
      --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      --env "GZ_PARTITION=greenhouse_$name"
      --mount "type=bind,source=$setup_dir,target=/etc/clearpath"
      --mount "type=bind,source=$config,target=/opt/greenhouse/config/selected.json,readonly"
      --mount "type=bind,source=$actuator_config,target=/opt/greenhouse/config/selected-actuator.json,readonly")
if [[ -n "$script" ]]; then
  args+=(--mount "type=bind,source=$(dirname -- "$script"),target=/opt/greenhouse/user_script"
         --env "GREENHOUSE_SCRIPT=/opt/greenhouse/user_script/$(basename -- "$script")")
fi
if [[ -n "$scenario" ]]; then
  args+=(--mount "type=bind,source=$scenario,target=/opt/greenhouse/config/scenario.json,readonly"
         --env GREENHOUSE_SCENARIO=/opt/greenhouse/config/scenario.json)
fi
if [[ -t 0 && -t 1 && "$mode" == run ]]; then args+=(-it); fi
if [[ "$software" == true ]]; then
  args+=(--env LIBGL_ALWAYS_SOFTWARE=1 --env GALLIUM_DRIVER=llvmpipe)
# Headless runs still render both gimbal cameras; on the CPU that halved the real-time factor.
elif [[ -d /dev/dri ]]; then
  for device in /dev/dri/renderD* /dev/dri/card*; do
    [[ -c "$device" ]] || continue
    args+=(--device "$device" --group-add "$(stat -c '%g' "$device")")
  done
fi
if [[ "$headless" == true ]]; then
  args+=(--env DISPLAY= --env QT_QPA_PLATFORM=offscreen)
else
  [[ "${DISPLAY:-}" =~ ^(:|unix:|unix/:)([0-9]+)(\.[0-9]+)?$ ]] ||
    die "a local X11/Xwayland DISPLAY such as :0 is required. For SSH/remote displays use --headless."
  display_number="${BASH_REMATCH[2]}"
  socket_dir="$(realpath -m -- /tmp/.X11-unix)"
  [[ -S "$socket_dir/X$display_number" ]] ||
    die "X11 socket $socket_dir/X$display_number is missing. Check that X11/Xwayland is running, or use --headless."
  args+=(--env "DISPLAY=$DISPLAY" --env QT_QPA_PLATFORM=xcb
         --env QT_X11_NO_MITSHM=1 --hostname "$(hostname)"
         --mount "type=bind,source=$socket_dir,target=/tmp/.X11-unix,readonly")
  auth="${XAUTHORITY:-}"
  if [[ -z "$auth" && -r "$HOME/.Xauthority" ]]; then auth="$HOME/.Xauthority"; fi
  if [[ -n "$auth" ]]; then
    [[ -f "$auth" && -r "$auth" ]] || die "XAUTHORITY is not a readable file: $auth"
    auth="$(realpath -- "$auth")"
    [[ "$auth" != *","* ]] || die "XAUTHORITY path cannot contain a comma"
    args+=(--env XAUTHORITY=/tmp/greenhouse.xauthority
           --mount "type=bind,source=$auth,target=/tmp/greenhouse.xauthority,readonly")
  fi
fi
# The container is on Docker's default bridge network; publish the Foxglove port to loopback only.
if [[ "$viz" == true ]]; then args+=(--publish 127.0.0.1:8767:8767); fi
args+=("$image" bash /opt/greenhouse/docker/start.sh "headless:=$headless" "rviz:=$rviz" "viz:=$viz" "gui_test:=$gui_test")
case "$mode" in
  smoke) args+=(--smoke-test) ;;
  generate) args+=(--generate-only) ;;
esac
args+=("${extra[@]}")
if [[ "$dry_run" == true ]]; then
  print_command "${args[@]}"
  exit 0
fi
printf 'Starting %s; Ctrl+C stops only this container. Setup: %s\n' "$name" "$setup_dir"
exec "${args[@]}"
