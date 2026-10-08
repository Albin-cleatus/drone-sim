#!/usr/bin/env bash
#
# Start ArduPilot SITL for a quadcopter and forward its MAVLink stream to a UDP
# port that sitl_mission.py can bind.
#
# Run this inside the environment where ArduPilot is built (WSL2/Ubuntu or
# native Linux). See README.md for the one-time build steps.
#
#   ./tools/start_sitl.sh                       # forward to 127.0.0.1:14550
#   GCS_HOST=172.20.16.1 ./tools/start_sitl.sh  # forward to the Windows host
#   NO_CONSOLE=1 NO_MAP=1 ./tools/start_sitl.sh # headless (no MAVProxy window)
#
set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
VEHICLE="${VEHICLE:-ArduCopter}"
FRAME="${FRAME:-quad}"
GCS_HOST="${GCS_HOST:-127.0.0.1}"
GCS_PORT="${GCS_PORT:-14550}"

# Default SITL location: CMAC, Canberra (ArduPilot's usual test site). This is a
# *name* from Tools/autotest/locations.txt, where CMAC is defined as
# -35.363261,149.165230,584,353.
#
# Raw "lat,lon,alt,heading" also works, but it must go to -l/--custom-location.
# -L/--location only accepts a name, and a name it cannot find is handed to an
# online geocoder, which exits the whole script when the lookup is refused -
# HTTP 403, or simply no network from inside WSL.
LOCATION="${LOCATION:-CMAC}"

if [ ! -d "$ARDUPILOT_DIR/ArduCopter" ]; then
    echo "ERROR: no ArduPilot checkout at $ARDUPILOT_DIR" >&2
    echo "       set ARDUPILOT_DIR, or clone it - see README.md" >&2
    exit 1
fi

# -L takes a name, -l takes literal coordinates. Pick the flag that matches
# what we were handed rather than assuming one of them.
if [[ "$LOCATION" == *,* ]]; then
    ARGS=(-v "$VEHICLE" -f "$FRAME" -l "$LOCATION")
else
    ARGS=(-v "$VEHICLE" -f "$FRAME" -L "$LOCATION")
fi
ARGS+=(--out="udp:${GCS_HOST}:${GCS_PORT}")

[ "${NO_CONSOLE:-0}" = "1" ] || ARGS+=(--console)
[ "${NO_MAP:-0}" = "1" ]     || ARGS+=(--map)

echo "Starting $VEHICLE ($FRAME) SITL"
echo "  location : $LOCATION"
echo "  MAVLink  : udp:${GCS_HOST}:${GCS_PORT}"
if [ "$GCS_HOST" != "127.0.0.1" ]; then
    echo
    echo "Reminder: sim_vehicle.py's own GCS (MAVProxy) also uses ports 14550-14552"
    echo "on its own machine. Point the Python client at this host/port."
fi
echo

cd "$ARDUPILOT_DIR/$VEHICLE"
exec python3 ../Tools/autotest/sim_vehicle.py "${ARGS[@]}" "$@"
