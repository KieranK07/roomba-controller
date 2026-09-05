#!/usr/bin/env bash
# Launch the Roomba control server.
set -euo pipefail
cd "$(dirname "$0")"

PORT="${ROOMBA_PORT:-}"
if [[ -z "$PORT" ]]; then
  PORT=$(ls /dev/cu.usbserial-* 2>/dev/null | head -1 || true)
fi
if [[ -z "$PORT" ]]; then
  echo "No /dev/cu.usbserial-* found. Is the Roomba cable plugged in?" >&2
  echo "Override with:  ROOMBA_PORT=/dev/cu.xxxx ./run.sh" >&2
  exit 1
fi

echo "Using serial port: $PORT"
exec ./.venv/bin/python server.py --port "$PORT" "$@"
