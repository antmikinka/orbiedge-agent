#!/usr/bin/env bash
set -eu
APP="/home/antmi/orbiedge-build/bin/Linux/OrbiEdgeApp"
LOG="/tmp/orbiedge-phase1-app.log"
PLANT="/mnt/c/Users/antmi/Downloads/orbiedge-agent/sim/fdir/canned_plant.py"
if [ ! -x "$APP" ]; then
  echo "phase1 FAIL missing $APP"
  exit 1
fi
: > "$LOG"
"$APP" > "$LOG" 2>&1 &
APP_PID=$!
cleanup() {
  if kill -0 "$APP_PID" 2>/dev/null; then
    kill "$APP_PID" 2>/dev/null || true
    wait "$APP_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
ready=0
for _ in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40; do
  if ! kill -0 "$APP_PID" 2>/dev/null; then
    echo "phase1 FAIL flight exited before the plant"
    cat "$LOG"
    exit 1
  fi
  if grep -q "listening on 5601" "$LOG"; then
    ready=1
    break
  fi
  sleep 0.25
done
if [ "$ready" -ne 1 ]; then
  echo "phase1 FAIL 5601 did not listen"
  cat "$LOG"
  exit 1
fi
python3 "$PLANT" --family thermal-critical
echo "phase1 plant_done"
