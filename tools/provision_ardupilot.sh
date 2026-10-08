#!/usr/bin/env bash
#
# provision_ardupilot.sh - build ArduPilot SITL inside Ubuntu on WSL2.
#
# Idempotent: re-running skips anything already done, so it is safe to retry
# after a failed or interrupted build.
#
# Run it as root, from Windows, with no password and no interactive first-run:
#
#     wsl -d Ubuntu-24.04 -u root -- \
#         bash /mnt/c/Users/Albin/drone-sim/tools/provision_ardupilot.sh
#
# On success the SITL binary is at ~ardupilot/build/sitl/bin/arducopter, and
# Tools/autotest/sim_vehicle.py can start it as the build user.
#
# Environment overrides:
#   BUILD_USER   UNIX account to create and build as      (default: albin)
#   TARGET       waf target                               (default: copter)
#   BOARD        waf board                                (default: sitl)
#   FORCE_PREREQS=1  run install-prereqs-ubuntu.sh even if it looks done

set -euo pipefail

BUILD_USER="${BUILD_USER:-albin}"
TARGET="${TARGET:-copter}"
BOARD="${BOARD:-sitl}"
REPO_URL="https://github.com/ArduPilot/ardupilot.git"
ARDUPILOT_DIR="/home/${BUILD_USER}/ardupilot"

# ArduPilot's own prereq script is the supported path, but it pulls in GUI
# packages (wxgtk and friends) that a headless SITL build does not need and that
# are occasionally missing from a given Ubuntu release. This is the subset that
# actually has to be present for `waf copter` to succeed.
FALLBACK_PKGS=(
    build-essential ccache g++ gcc make git pkg-config
    python3 python3-dev python3-pip python3-venv python3-setuptools
    python3-wheel python3-future python3-lxml python3-numpy
    libtool automake autoconf libexpat1-dev libxml2-dev gawk
)

log()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33mWARN: %s\033[0m\n' "$*" >&2; }
die()  { printf '\033[1;31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

[[ $EUID -eq 0 ]] || die "must run as root - use: wsl -d <distro> -u root -- bash $0"

# --------------------------------------------------------------------------- #
# 1. Base packages, as root.
# --------------------------------------------------------------------------- #

log "apt update"
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq

log "installing base packages"
apt-get install -y -qq --no-install-recommends \
    git python3 python3-venv python3-dev build-essential ccache g++ make \
    ca-certificates sudo locales

# --------------------------------------------------------------------------- #
# 2. The build user.
# --------------------------------------------------------------------------- #

if id "$BUILD_USER" >/dev/null 2>&1; then
    log "user '$BUILD_USER' already exists"
else
    log "creating user '$BUILD_USER'"
    # No password login is ever needed: `wsl -u` switches to the user directly,
    # and sudo is passwordless below so ArduPilot's own scripts run unattended.
    adduser --disabled-password --gecos "" "$BUILD_USER"
fi

log "granting '$BUILD_USER' passwordless sudo"
# ArduPilot's install-prereqs-ubuntu.sh calls sudo internally. WSL has no
# interactive console here, so a password prompt would just hang the build.
printf '%s ALL=(ALL) NOPASSWD:ALL\n' "$BUILD_USER" > "/etc/sudoers.d/90-${BUILD_USER}-nopasswd"
chmod 0440 "/etc/sudoers.d/90-${BUILD_USER}-nopasswd"
visudo -cf "/etc/sudoers.d/90-${BUILD_USER}-nopasswd" >/dev/null \
    || die "generated sudoers file is invalid"

# Make the user the default for interactive `wsl` sessions, so a plain
# `wsl -d <distro>` lands somewhere sane instead of as root.
if ! grep -qs "^default=${BUILD_USER}$" /etc/wsl.conf 2>/dev/null; then
    log "setting '$BUILD_USER' as the default WSL user"
    cat > /etc/wsl.conf <<EOF
[user]
default=${BUILD_USER}

[boot]
systemd=true
EOF
fi

# --------------------------------------------------------------------------- #
# 3. ArduPilot, as the build user.
# --------------------------------------------------------------------------- #

log "building ArduPilot SITL as '$BUILD_USER' (this is the slow part: 10-30 min)"

# The user-side script goes to a real temp file rather than being piped into
# `su -c 'bash -s'`: stdin handling through su is fiddly, and a file on disk can
# be re-read or re-run after a failure. Quoted heredoc, so nothing expands here.
INNER_SCRIPT="$(mktemp /tmp/provision-ardupilot.XXXXXX.sh)"
trap 'rm -f "$INNER_SCRIPT"' EXIT
cat > "$INNER_SCRIPT" <<'USERSCRIPT'
set -euo pipefail

log()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33mWARN: %s\033[0m\n' "$*" >&2; }

cd "$HOME"

if [[ -d "$ARDUPILOT_DIR/.git" ]]; then
    log "ardupilot already cloned; updating submodules"
    git -C "$ARDUPILOT_DIR" submodule update --init --recursive
else
    log "cloning ardupilot (with submodules - this is a large checkout)"
    git clone --recurse-submodules "$REPO_URL" "$ARDUPILOT_DIR"
fi

cd "$ARDUPILOT_DIR"

# ------------------------------------------------------------------------- #
# Prerequisites. Prefer ArduPilot's own installer; fall back to the known-good
# package subset if it trips over a GUI package or a release it does not know.
# ------------------------------------------------------------------------- #
PREREQ_STAMP="$HOME/.ardupilot-prereqs-ok"

if [[ -f "$PREREQ_STAMP" && "${FORCE_PREREQS:-0}" != "1" ]]; then
    log "prerequisites already installed ($PREREQ_STAMP); skipping"
else
    log "running ArduPilot's install-prereqs-ubuntu.sh"
    if Tools/environment_install/install-prereqs-ubuntu.sh -y; then
        touch "$PREREQ_STAMP"
    else
        warn "install-prereqs-ubuntu.sh failed - installing the minimal subset instead."
        warn "That is fine for a headless SITL build; it skips the GUI/MAVProxy extras."
        sudo apt-get install -y -qq --no-install-recommends \
            build-essential ccache g++ gcc make git pkg-config \
            python3 python3-dev python3-pip python3-venv python3-setuptools \
            python3-wheel python3-future python3-lxml python3-numpy \
            libtool automake autoconf libexpat1-dev libxml2-dev gawk
        touch "$PREREQ_STAMP"
    fi

    # install-prereqs-ubuntu.sh appends PATH and python bits to ~/.profile.
    # A `su -c` shell is not a login shell, so source it explicitly.
    # shellcheck disable=SC1090
    [[ -f "$HOME/.profile" ]] && . "$HOME/.profile" || true
fi

log "configured board check"
./waf configure --board "$BOARD"

log "compiling target '$TARGET'"
./waf "$TARGET"

log "build artefacts"
ls -la "build/$BOARD/bin/"
USERSCRIPT

chmod 0755 "$INNER_SCRIPT"
chown "$BUILD_USER" "$INNER_SCRIPT"

# runuser rather than `su -`: the environment is explicit and stdin is left
# alone. The inner script sources ~/.profile itself where it needs the PATH
# entries that install-prereqs-ubuntu.sh writes.
if command -v runuser >/dev/null 2>&1; then
    runuser -u "$BUILD_USER" -- env \
        HOME="/home/$BUILD_USER" \
        BUILD_USER="$BUILD_USER" \
        TARGET="$TARGET" \
        BOARD="$BOARD" \
        ARDUPILOT_DIR="$ARDUPILOT_DIR" \
        REPO_URL="$REPO_URL" \
        FORCE_PREREQS="${FORCE_PREREQS:-0}" \
        bash "$INNER_SCRIPT"
else
    warn "runuser not found; falling back to su"
    su - "$BUILD_USER" -c "
        BUILD_USER='$BUILD_USER' TARGET='$TARGET' BOARD='$BOARD' \
        ARDUPILOT_DIR='$ARDUPILOT_DIR' REPO_URL='$REPO_URL' \
        FORCE_PREREQS='${FORCE_PREREQS:-0}' \
        bash '$INNER_SCRIPT'"
fi

# --------------------------------------------------------------------------- #
# 4. Verify, as root, from a clean environment.
# --------------------------------------------------------------------------- #

log "verifying the SITL binary"
BINARY="${ARDUPILOT_DIR}/build/${BOARD}/bin/arducopter"
[[ -x "$BINARY" ]] || die "expected binary not found or not executable: $BINARY"
file "$BINARY" | sed 's/^/    /'

log "done"
cat <<EOF

    ArduPilot SITL is built.

      binary     : $BINARY
      sim_vehicle: ${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py
      built as   : $BUILD_USER

    Start it (in a WSL shell, or straight from Windows):

      wsl -d Ubuntu-24.04 -u $BUILD_USER -- bash -lc \\
          'cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f quad \\
               -L CMAC --console --map \\
               --out=udp:127.0.0.1:14550'

    then fly the mission from the Windows side:

      cd C:\\Users\\Albin\\drone-sim
      .\\.venv\\Scripts\\Activate.ps1
      python sitl_mission.py --connect udpin:0.0.0.0:14550

EOF
