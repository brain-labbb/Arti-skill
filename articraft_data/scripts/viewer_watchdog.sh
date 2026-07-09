#!/usr/bin/env bash
# Viewer watchdog: keeps the Articraft viewer on 127.0.0.1:8765 alive.
#
# Health-checks GET /health every INTERVAL seconds with a hard timeout. If the
# server is unreachable OR wedged (accepts the connection but never answers)
# for FAILS consecutive checks, it kills the process group and relaunches it.
#
# This is a safety net, not a fix: the real hang causes were addressed by
# moving compile work to a subprocess and offloading blocking store calls off
# the event loop. The watchdog just guarantees "when I open it, it's alive."
#
# Usage:
#   nohup scripts/viewer_watchdog.sh > logs/viewer/watchdog.log 2>&1 &
#   scripts/viewer_watchdog.sh --once     # start-if-down + single health check, then exit
#
# Env overrides: VIEWER_HOST VIEWER_PORT INTERVAL TIMEOUT FAILS
set -u

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

HOST="${VIEWER_HOST:-127.0.0.1}"
PORT="${VIEWER_PORT:-8765}"
INTERVAL="${INTERVAL:-15}"     # seconds between health checks
TIMEOUT="${TIMEOUT:-8}"        # per-check hard timeout (a wedged server hangs, not refuses)
FAILS="${FAILS:-3}"           # consecutive failures before restart
PYTHON="$REPO_DIR/.venv/bin/python"
LOG_DIR="$REPO_DIR/logs/viewer"
UVICORN_LOG="$LOG_DIR/uvicorn.log"
PATTERN="uvicorn viewer.api.app:app"

mkdir -p "$LOG_DIR"

log() { printf '%s watchdog: %s\n' "$(date '+%Y-%m-%dT%H:%M:%S')" "$*"; }

viewer_pid() { pgrep -f "$PATTERN" | head -1; }

healthy() {
  local code
  code="$(curl -sS -m "$TIMEOUT" -o /dev/null -w '%{http_code}' "http://$HOST:$PORT/health" 2>/dev/null)"
  [ "$code" = "200" ]
}

start_viewer() {
  log "starting viewer on $HOST:$PORT"
  nohup "$PYTHON" -m uvicorn viewer.api.app:app --host "$HOST" --port "$PORT" \
    >> "$UVICORN_LOG" 2>&1 &
  disown
}

restart_viewer() {
  local pid; pid="$(viewer_pid)"
  if [ -n "$pid" ]; then
    log "killing wedged viewer pid=$pid (threads=$(ls /proc/$pid/task 2>/dev/null | wc -l))"
    kill "$pid" 2>/dev/null
    for _ in 1 2 3 4 5; do
      sleep 1
      [ -d "/proc/$pid" ] || break
    done
    [ -d "/proc/$pid" ] && { log "force-killing pid=$pid"; kill -9 "$pid" 2>/dev/null; sleep 1; }
  fi
  start_viewer
}

check_once() {
  if [ -z "$(viewer_pid)" ]; then
    log "viewer not running"
    start_viewer
    return
  fi
  if healthy; then
    consecutive=0
  else
    consecutive=$((consecutive + 1))
    log "health check failed ($consecutive/$FAILS)"
    if [ "$consecutive" -ge "$FAILS" ]; then
      restart_viewer
      consecutive=0
    fi
  fi
}

consecutive=0

if [ "${1:-}" = "--once" ]; then
  check_once
  exit 0
fi

log "watchdog up (interval=${INTERVAL}s timeout=${TIMEOUT}s fails=${FAILS})"
while true; do
  check_once
  sleep "$INTERVAL"
done
