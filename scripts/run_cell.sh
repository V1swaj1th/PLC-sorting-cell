#!/usr/bin/env bash
# Run this on the HOST (not inside the container). It starts the sorting cell inside the ROS
# container and opens the scene dashboard in this computer's browser as soon as it answers.
#
#   ./scripts/run_cell.sh                       # stand-alone cell
#   ./scripts/run_cell.sh use_plc:=true         # any sorting.launch.py argument can be added
#
# Settings (environment variables): CONTAINER (default ros2-gazebo), PORT (default 8765),
# WS = workspace inside the container (default /root/sorting_cell_ws), NO_BROWSER=1 to skip opening.
set -euo pipefail
CONTAINER="${CONTAINER:-ros2-gazebo}"
PORT="${PORT:-8765}"
WS="${WS:-/root/sorting_cell_ws}"

if ! command -v docker >/dev/null 2>&1; then
  echo "run_cell.sh: docker not found. Run this script on the host, not inside the container." >&2
  exit 1
fi
if ! running=$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null); then
  echo "run_cell.sh: there is no container called '$CONTAINER' (set CONTAINER=name)." >&2
  exit 1
fi
if [ "$running" != "true" ]; then
  echo "Starting container $CONTAINER ..."
  docker start "$CONTAINER" >/dev/null
fi

# Where the browser on this computer can reach the dashboard.
mode=$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$CONTAINER")
if [ "$mode" = "host" ]; then
  addr="127.0.0.1"; url="http://localhost:$PORT"; bind="127.0.0.1"
else
  addr=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}} {{end}}' "$CONTAINER" | awk '{print $1}')
  if [ -z "$addr" ]; then
    echo "run_cell.sh: container network mode is '$mode' and it has no IP address I can use." >&2
    exit 1
  fi
  url="http://$addr:$PORT"; bind="0.0.0.0"   # listen on the container's own address
fi
echo "Container network: $mode -> dashboard at $url"

port_open() { (exec 3<>"/dev/tcp/$addr/$PORT") 2>/dev/null; }
if port_open; then
  echo "run_cell.sh: something already answers on $url. Is the cell already running? Stop it first." >&2
  exit 1
fi

open_url() {
  local b
  for b in xdg-open "${BROWSER:-}" chromium firefox google-chrome brave chromium-browser; do
    if [ -n "$b" ] && command -v "$b" >/dev/null 2>&1; then
      setsid "$b" "$url" </dev/null >/dev/null 2>&1 &
      echo "Opened the dashboard: $url"
      return 0
    fi
  done
  echo "No browser command found. Open $url yourself."
}

# Wait (up to 90 s) for the dashboard to answer, then open it. Runs beside the launch.
watcher() {
  local i
  for ((i = 0; i < 180; i++)); do
    if port_open; then
      [ "${NO_BROWSER:-0}" = "1" ] && echo "Dashboard is up: $url" || open_url
      return 0
    fi
    sleep 0.5
  done
  echo "The dashboard did not answer on $url within 90 s. Check the launch output above." >&2
}
watcher &
watcher_pid=$!
trap 'kill "$watcher_pid" 2>/dev/null || true' EXIT

args=""
for a in "$@"; do args+=" $(printf '%q' "$a")"; done
inner="source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && exec ros2 launch sorting_cell_bringup sorting.launch.py dashboard_host:=$bind dashboard_port:=$PORT$args"
tty_flag="-i"; [ -t 0 ] && tty_flag="-it"
# bash -i loads the container's ~/.bashrc, so the display settings RViz needs are the usual ones.
docker exec $tty_flag "$CONTAINER" bash -ic "$inner"
