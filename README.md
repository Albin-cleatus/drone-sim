# drone-sim — ArduPilot SITL quadcopter mission + telemetry logger

A local ArduPilot SITL (software-in-the-loop) setup for a quadcopter, plus a
`pymavlink` client that flies a short autonomous mission and logs everything to
CSV.

The mission the script flies:

1. Connect to the simulated vehicle over UDP and wait for a usable GPS fix and
   EKF solution.
2. Switch to **GUIDED**, arm, and take off to **5 m**.
3. Navigate three GPS waypoints (25 m N, then 25 m N + 25 m E, then 25 m E of the
   takeoff point).
4. Hold **LOITER**, then **RTL** and wait for auto-disarm.

Throughout, it logs telemetry, the autopilot's reported CPU load, the measured
MAVLink packet rate, and battery state to a timestamped CSV.

```
drone-sim/
├── sitl_mission.py        # the mission driver + telemetry logger
├── teleop_keyboard.py     # keyboard teleoperation (velocity setpoints)
├── requirements.txt
├── README.md
├── .gitattributes         # pins LF endings, so WSL can execute the .sh files
├── logs/                  # telemetry CSVs land here (gitignored)
└── tools/
    ├── start_sitl.sh           # launches sim_vehicle.py with the right --out
    ├── fake_sitl.py            # offline stub vehicle, for testing without ArduPilot
    ├── wsl_bootstrap.ps1       # Windows: enable WSL2 (run once, elevated)
    ├── provision_ardupilot.sh  # WSL: create build user + build ArduPilot SITL
    └── post_reboot_setup.ps1   # runs provision_ardupilot.sh unattended at logon
```

---

## 1. Prerequisites

| Component | Notes |
|---|---|
| ArduPilot SITL | Built and run inside **WSL2 + Ubuntu** (recommended on Windows) or native Linux/macOS. |
| Python 3.8+ | With `pymavlink`. |
| `pymavlink` | `pip install -r requirements.txt` |

ArduPilot's documentation for Windows points at **WSL2** ("SITL setup on Windows
using WSL"), and that is the route this README follows. Two alternatives exist:

- **Native Windows or Cygwin builds** of the SITL binary are possible, but they
  are not the documented path, and the helper scripts here assume a POSIX shell.
- **Mission Planner's built-in simulator** bundles a prebuilt ArduPilot and can
  run it from its *Simulation* tab. It manages its own MAVLink ports, so point
  `--connect` at whatever port you forward to — Mission Planner's Mavlink Mirror
  can re-broadcast the stream to a UDP client.

Everything below assumes the WSL2 route unless stated otherwise.

---

## 2. One-time setup

### Windows: install WSL2 + Ubuntu

Two steps, split by privilege — only the first needs Administrator.

**1. Enable WSL2.** In an **Administrator** PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File tools\wsl_bootstrap.ps1
```

The same thing by hand:

```powershell
wsl --install --no-distribution
```

Both enable the `Microsoft-Windows-Subsystem-Linux` and `VirtualMachinePlatform`
optional features and **require a reboot**. The script writes
`tools/wsl_bootstrap.log` as it goes, and refuses to run unelevated.

**2. Register Ubuntu.** After rebooting, in a *normal* (non-elevated)
PowerShell — adding a distro does not need Administrator:

```powershell
wsl --install -d Ubuntu-24.04 --no-launch
```

`--no-launch` matters: it registers the distro without starting its first-run
wizard, so you are never prompted to invent a UNIX username and password. The
provisioning step below creates the build account for you instead.

> For the smoothest networking, put this in `C:\Users\<you>\.wslconfig` and run
> `wsl --shutdown` once. Mirrored networking makes `localhost` work in both
> directions between Windows and WSL, so no IP juggling is needed.
>
> ```ini
> [wsl2]
> networkingMode=mirrored
> ```

### Inside Ubuntu (WSL or native Linux): build ArduPilot SITL

Once Ubuntu is registered, one command from Windows does the whole build. It
runs as root inside the distro, so it needs no password and no interactive
login:

```powershell
wsl -d Ubuntu-24.04 -u root -- `
    bash /mnt/c/Users/Albin/drone-sim/tools/provision_ardupilot.sh
```

It is idempotent — re-running it after a failure or an interruption skips
whatever already succeeded. What it does:

- installs the base packages, then creates the build user (default `albin`)
  with passwordless `sudo`, and sets it as the default WSL user
- clones `ardupilot` **with submodules** and runs ArduPilot's own
  `install-prereqs-ubuntu.sh -y`, falling back to a minimal package subset if
  that trips over a GUI dependency
- runs `./waf configure --board sitl` and `./waf copter`
- verifies `build/sitl/bin/arducopter` exists and prints how to launch it

Override the build user or target with the environment, e.g.
`BUILD_USER=alice TARGET=copter wsl ... bash ...`.

<details>
<summary>The same steps by hand</summary>

```bash
sudo apt update
sudo apt install -y git python3 python3-pip python3-venv python3-dev \
                    build-essential ccache g++ make

git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot

# Installs ArduPilot's own build dependencies and writes them into ~/.profile
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Configure and build the SITL target for Copter
./waf configure --board sitl
./waf copter
```

</details>

The Copter build takes a while (10–30 min depending on the machine) and needs
several GB of disk. It only needs doing once; `sim_vehicle.py` rebuilds
incrementally after that.

### Install the Python client dependencies

Inside the same Ubuntu environment (simplest — everything on one machine):

```bash
cd /mnt/c/Users/Albin/drone-sim          # or wherever you cloned this
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

If you would rather run the Python client on **Windows** (not WSL), do this
instead in PowerShell:

```powershell
cd C:\Users\Albin\drone-sim
py -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

---

## 3. Running it

You need two terminals: one for the simulator, one for the mission client.

### Terminal 1 — start SITL

**If both live in Ubuntu/WSL**, the helper script does the right thing:

```bash
cd /mnt/c/Users/Albin/drone-sim
./tools/start_sitl.sh
```

**If you would rather stay on the Windows side**, you can launch it through WSL
in one line, without opening a WSL shell first:

```powershell
wsl -d Ubuntu-24.04 -u albin -- bash -lc `
  'cd ~/ardupilot/ArduCopter && exec sim_vehicle.py -v ArduCopter -f quad `
  -L CMAC --console --map --out=udp:127.0.0.1:14550'
```

`bash -lc` matters there — `sim_vehicle.py` reaches `PATH` via the `~/.profile`
entries that ArduPilot's prereq script writes, and only a login shell reads it.

That is equivalent to:

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f quad \
    -L CMAC \
    --console --map \
    --out=udp:127.0.0.1:14550
```

`-L CMAC` names a site from `Tools/autotest/locations.txt`;

```bash
sim_vehicle.py --list-locations     # see the ~120 built-in sites
```

`CMAC` is Canberra, defined as `-35.363261,149.165230,584,353`. To use your own
coordinates instead, note that they go to the **lowercase** `-l`:

```bash
sim_vehicle.py -v ArduCopter -f quad -l -35.363261,149.165230,584,353
```

`-L` (uppercase) accepts only a *name*, and a name it does not recognise is
sent to an online geocoder — see the troubleshooting entry below, because that
fails hard when the machine has no route to the geocoder.

Wait for MAVProxy to report a heartbeat and for `GPS: 3D fix`. The simulated GPS
takes roughly 10–30 s to converge — the mission script waits for it.

`--console` gives you the MAVProxy command line (try `status`, `wp`, `param
show BATT_MONITOR`); `--map` opens a moving map. Add `NO_CONSOLE=1 NO_MAP=1` to
run the helper headless.

### Terminal 2 — fly the mission

```bash
cd /mnt/c/Users/Albin/drone-sim
source .venv/bin/activate
python3 sitl_mission.py --connect udpin:0.0.0.0:14550
```

On Windows PowerShell, the equivalent is:

```powershell
cd C:\Users\Albin\drone-sim
.\.venv\Scripts\Activate.ps1
python sitl_mission.py --connect udpin:0.0.0.0:14550
```

You should see the mode change to GUIDED, arming, the climb to 5 m, three
waypoint legs, a loiter hold, then RTL and disarm. The CSV path is printed at
startup and summarised at the end. A healthy run of the defaults looks like
this — values from an actual recorded flight:

```
ready: fix=6 sats=10
mode -> GUIDED (custom_mode=4)
airborne at 4.7 m
wp1 reached (1.35 m from target)
wp2 reached (1.24 m from target)
wp3 reached (1.49 m from target)
hold complete: alt 5.0 m, batt 12.31 V (93 %)
mode -> RTL (custom_mode=6)
alt  15.0 m                      <- RTL climbs to RTL_ALT before returning
SIM Hit ground at 0.501328 m/s   <- LAND_SPEED, i.e. nominal
landed and disarmed
duration      : 119.0 s
mavlink       : 4886 msgs, 188.5 KiB, 41 msgs/s average
```

Two outputs are easy to misread:

- **Waypoint distances are acceptance, not accuracy.** `--wp-radius` defaults to
  1.5 m, so "reached" means the vehicle got *inside that circle*; the figures
  above are where each leg ended, not how tightly the controller can converge.
  Tighten `--wp-radius` if you want it to close further.
- **`SIM Hit ground at ...` is not a crash report.** SITL prints touchdown
  velocity for every landing. 0.50 m/s is `LAND_SPEED` doing its job; a figure
  in the metres-per-second range means something is genuinely wrong.

> **Why LOITER can sink on its own.** Unlike GUIDED, LOITER takes its climb
> rate from the pilot's throttle stick (`ModeLoiter::run()` calls
> `get_pilot_desired_climb_rate_ms()`), not from a GCS altitude target. With no
> transmitter attached the stick reads minimum, and because `PILOT_SPD_DN`
> defaults to 0 it falls back to `PILOT_SPD_UP` — a commanded 2.5 m/s descent
> that will fly the vehicle into the ground. `sitl_mission.py` compensates by
> streaming `RC_CHANNELS_OVERRIDE` with the sticks centred for the whole hold,
> re-sent at 2 Hz to stay inside `RC_OVERRIDE_TIME` (3 s). Any LOITER hold you
> write against SITL needs the same override.

### Alternative: skip MAVProxy (TCP)

`sim_vehicle.py` starts MAVProxy alongside the vehicle, and MAVProxy is the layer
that owns the UDP `--out` streams. If you would rather not deal with UDP port
selection at all, drop MAVProxy and talk to the simulated vehicle directly:

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f quad --no-mavproxy
```

SITL listens on TCP 5760 (check the startup output if you have customised
`--serial0`), so the client connects with TCP instead:

```bash
python3 sitl_mission.py --connect tcp:127.0.0.1:5760
```

You lose the MAVProxy console and map, but there is no UDP port juggling and
nothing else is competing for the link.

### Alternative: keyboard teleoperation

`teleop_keyboard.py` flies the vehicle by hand rather than by mission, streaming
body-frame velocity setpoints while a key is held:

```bash
python3 teleop_keyboard.py       # connects to udp:127.0.0.1:14550
```

| Key | Command |
|---|---|
| `W` / `S` | forward / backward, 2.5 m/s |
| `A` / `D` | left / right, 2.5 m/s |
| `↑` / `↓` | climb / descend, 1.0 m/s |
| `←` / `→` | yaw left / right, 0.5 rad/s |
| `space` | hold — sends a zero vector while held |
| `Esc` | switch to LAND and exit |

It waits for a heartbeat, selects GUIDED, arms, commands a takeoff to 3 m, then
sends `SET_POSITION_TARGET_LOCAL_NED` in `MAV_FRAME_BODY_NED` at 10 Hz. The
`type_mask` is `0b0000011111000111`, which ignores position, acceleration, force
and yaw, so only the velocity and yaw-rate fields are honoured. ArduPilot
accepts the body frame here and rotates the vector to earth coordinates itself
(`GCS_MAVLink_Copter.cpp:980`), rather than expecting you to do it.

Worth knowing before you rely on it:

- **Body frame means `W` is wherever the nose points**, not north. Yaw with the
  arrow keys and the direction of travel follows.
- **Setpoints expire.** GUIDED drops an unrefreshed target after `GUID_TIMEOUT`
  (3 s), so the 10 Hz loop is what keeps the vehicle moving. Anything that blocks
  that loop for three seconds stops the aircraft.
- **The takeoff is a fixed 7-second sleep**, not a wait for altitude. On a slow
  start your first keypresses can arrive before the vehicle is airborne.
- It reads keys through `pynput` and needs a real console; it will not work
  through a pipe or a non-interactive shell.

---

## 4. If the client is on Windows and SITL is in WSL

With WSL's default NAT networking, `127.0.0.1` inside WSL means WSL itself — not
your Windows host. Two fixes:

**A. Mirrored networking** (see `.wslconfig` above). Then `--out=udp:127.0.0.1:14550`
works as-is and the client's `udpin:0.0.0.0:14550` receives it.

**B. Point SITL at the Windows host**, using the host's IP as seen from WSL:

```bash
WIN_IP=$(ip route show default | awk '{print $3}')
GCS_HOST=$WIN_IP ./tools/start_sitl.sh
```

You will also need to let that UDP traffic through Windows Firewall. In an
**Administrator** PowerShell:

```powershell
netsh advfirewall firewall add rule name="ArduPilot SITL UDP 14550" `
    dir=in action=allow protocol=UDP localport=14550
```

The client still binds with `--connect udpin:0.0.0.0:14550`.

> Note: MAVProxy forwards the stream to `127.0.0.1:14550` and `:14551` by
> default, which is exactly why the client binds 14550 — that pairing is the
> intended one. The only conflict is if something else on the client's machine
> (Mission Planner, QGroundControl) is already bound to 14550. In that case give
> the client its own port: `--out=udp:127.0.0.1:14600` on the SITL side and
> `--connect udpin:0.0.0.0:14600` on the client side.

---

## 5. What gets logged

One CSV per run, `logs/telemetry_<timestamp>.csv`, sampled at 5 Hz by default.
The timestamp in the filename is **local** time, while the `timestamp_utc`
column inside the file is UTC — so the two disagree unless the machine runs on
UTC.

| Group | Columns |
|---|---|
| Timing | `timestamp_utc`, `t_rel_s`, `link` |
| State | `mode`, `armed`, `wp_index`, `wp_dist_m` |
| Position | `lat`, `lon`, `alt_rel_m`, `alt_msl_m` |
| Velocity | `vn_mps`, `ve_mps`, `vd_mps`, `groundspeed_mps` |
| Attitude | `roll_deg`, `pitch_deg`, `yaw_deg`, `heading_deg`, `throttle_pct`, `climb_mps` |
| GPS / EKF | `gps_fix`, `gps_sats`, `gps_eph`, `ekf_ok` |
| CPU | `cpu_load_pct` (autopilot `SYS_STATUS.load`), `host_cpu_pct` (SITL process, needs `--sitl-pid`) |
| Link rates | `rx_rate_hz`, `rx_kbps`, `sys_drop_rate_pct`, `sys_errors_comm` |
| Battery | `batt_voltage_v`, `batt_voltage_pack_v`, `batt_current_a`, `batt_remaining_pct`, `batt_temp_c` |

Missing values are written as empty cells, not `nan`.

**On the CPU number.** `cpu_load_pct` is what the *autopilot* reports about
itself (`SYS_STATUS.load`). Under SITL that value comes from the simulated
scheduler, not flight hardware, and in practice it is always **0** — a measured
flight logged 0 for 392 of 393 samples. It is not a usable measurement. For a
number you can act on, sample the real SITL process:

```bash
# find the pid of the SITL binary (not sim_vehicle.py)
pgrep -f arducopter
python3 sitl_mission.py --sitl-pid <pid>
```

That needs `psutil` (`pip install psutil`) and fills in `host_cpu_pct`. With
mirrored WSL networking, a Windows-side client can sample a WSL process only if
the pid is visible to it, so this works best with everything inside WSL.

**On packet rates.** `rx_rate_hz` / `rx_kbps` are measured on the client: the
receive loop counts every frame and every byte, and the totals are divided by a
wall-clock window each second. `sys_drop_rate_pct` and `sys_errors_comm` are the
autopilot's own view of the telemetry link, from `SYS_STATUS`.

---

## 6. Command-line options

```
--connect STR            pymavlink connection string (default udpin:0.0.0.0:14550)
--connect-timeout SEC    wait for the first heartbeat (default 60)
--no-stream-setup        skip the SET_MESSAGE_INTERVAL requests

--takeoff-alt M          takeoff altitude (default 5.0)
--waypoints N,E;N,E;...  metres north,east of the takeoff point
                         (default "25,0;25,25;0,25")
--wp-radius M            horizontal acceptance radius (default 1.5)
--wp-timeout SEC         per-waypoint timeout (default 120)
--takeoff-timeout SEC    wait for the climb to complete (default 60)
--ready-timeout SEC      wait for 3D fix + EKF (default 180)
--yaw-to-waypoint        also command yaw, so the nose points along track
--loiter-seconds SEC     hold LOITER after the last waypoint (default 5)
--end-action {rtl,land,none}   what to do after the hold (default rtl)
--end-timeout SEC        wait for the end action to land + disarm (default 180)
--skip-arming-checks     set ARMING_CHECK=0 before arming

--log-csv PATH           CSV path (default logs/telemetry_<timestamp>.csv)
--log-hz HZ              CSV sample rate (default 5)
--sitl-pid PID           sample host CPU for this process (needs psutil)
```

Examples:

```bash
# higher, with custom waypoints that form a square
python3 sitl_mission.py --takeoff-alt 15 \
    --waypoints "20,0;20,20;0,20;0,0"

# nose points along track, land instead of RTL
python3 sitl_mission.py --yaw-to-waypoint --end-action land

# log at 20 Hz to a specific file
python3 sitl_mission.py --log-hz 20 --log-csv logs/run1.csv
```

---

## 7. Testing without ArduPilot

`tools/fake_sitl.py` is a small stub that speaks just enough MAVLink to run the
client offline — useful for checking your Python environment before committing
to a 30-minute ArduPilot build, and for exercising the failure paths.

Terminal 1:

```bash
python3 tools/fake_sitl.py --out 127.0.0.1:14550 --bind 0.0.0.0:14551
```

Terminal 2:

```bash
python3 sitl_mission.py --connect udpin:0.0.0.0:14550
```

It simulates a GPS that takes 10 s to lock, a draining 4S battery, arming checks
(including refusals), and guided setpoints that expire after 3 s — the same
`GUID_TIMEOUT` behaviour as ArduPilot, so the client's setpoint re-sending is
genuinely exercised. It is **not** a flight dynamics model: it translates toward
whatever setpoint it was last given at a fixed rate.

Useful flags:

```bash
--lock-seconds 25     # slow GPS lock, to watch the readiness wait
--deny-arms 2         # refuse the first two arm attempts
--batt-mah 1000       # drain the battery faster
```

---

## 8. Troubleshooting

**`no autopilot heartbeat within 60s`**
SITL is not running, or `--out` is pointing somewhere the client is not
listening. Check that `sim_vehicle.py` printed its output port, and that the
client's `--connect` uses the same host and port. If the client is on Windows
and SITL in WSL, see section 4.

If you are connecting over **TCP** and the handshake itself succeeded, the cause
is usually the next entry instead: a SITL serial port takes one client, and
yours is queued behind a stale one.

**The client connects to TCP 5760 and then receives nothing**
A SITL serial port accepts exactly one TCP client. `_check_connection()` in
`libraries/AP_HAL_SITL/UARTDriver.cpp` returns early on `_connected` — the
source comment is literally `// we only want 1 connection at a time`. A second
client still completes the TCP handshake, because the kernel queues it in the
listen backlog, so the connection *looks* successful while nothing is ever read
from it. A stale MAVProxy, or an earlier client run that never exited, is
enough. Confirm with:

```bash
ss -ltnp | grep 5760      # Recv-Q > 0 on a LISTEN socket = a client is stuck
ss -tnp  | grep 5760      # which pid actually holds the accepted socket
```

Fix it by dropping the stale client, or by not competing for the port at all —
see the next entry.

**Port collisions, and running a second vehicle (`-I`)**

Two `sim_vehicle.py` instances both default to the same ports, so the second one
fights the first for 5760 and its clients starve. The `-I` flag shifts the whole
block, giving you a vehicle nobody else is talking to:

```bash
sim_vehicle.py -v ArduCopter -f quad -L CMAC --no-mavproxy -I 5
```

| Instance | SERIAL0 | SERIAL1 | SERIAL2 |
|---|---|---|---|
| `-I 0` (default) | 5760 | 5762 | 5763 |
| `-I 5` | 5810 | 5812 | 5813 |

Then point the client at the matching port: `--connect tcp:127.0.0.1:5810`. The
offset is 50 ports per instance. Before launching, check what is already
listening:

```bash
ss -ltnp | grep -E '576[0-9]|581[0-9]'
```

Two SITL instances plus MAVProxy will also put real load on the machine —
`uptime` showed a load average near 4.7 with two running — which matters for a
simulation whose whole purpose is timing fidelity.

**`could not open udpin:0.0.0.0:14550: [Errno 10048]` / `Address already in use`**
Something else already bound the port — MAVProxy started with
`--master=udp:...`, or a previous run of the client that did not exit. Pick
another port with `--out=udp:127.0.0.1:14600` on the SITL side and
`--connect udpin:0.0.0.0:14600` on the client side.

**Arming fails, script exits after 4 attempts**
The script prints the vehicle's `STATUSTEXT` lines, which carry the actual
PreArm reason (e.g. `PreArm: GPS: Bad fix`, `PreArm: EKF2 still initialising`).
Usually it just needs more time — the default `--ready-timeout 180` should cover
SITL. To fly regardless: `--skip-arming-checks`.

**The copter stops dead in the middle of a leg**
That is a guided setpoint that timed out. ArduPilot expires a guided target after
`GUID_TIMEOUT` (default 3 s); the script re-sends at 2 Hz, so seeing this means
the link dropped. Check `link` and `rx_rate_hz` in the CSV around that timestamp.

**Mode never changes to GUIDED**
Check `--mode` support: some builds/vehicle types do not have GUIDED. The script
prints the vehicle's mode map at connect time.

**`geocoder access denied (HTTP 403)` and SITL exits straight away**
`-L/--location` takes a *name* from `Tools/autotest/locations.txt`. A name it
cannot find there is sent to the online geocoder at
`nominatim.openstreetmap.org`, and when that lookup is refused — no route to
the internet from inside WSL, a proxy, or simple rate limiting — `sim_vehicle.py`
calls `sys.exit(1)` and the vehicle never starts. Handing raw coordinates to
`-L` triggers this every time, because a coordinate string is not a name.

Use a built-in name (`-L CMAC`), or the lowercase flag for literal coordinates
(`-l -35.363261,149.165230,584,353`). `tools/start_sitl.sh` chooses the right
flag based on what you give it.

**Waypoints are in the wrong place**
Waypoints are offsets in metres from wherever the vehicle was when it armed, not
absolute coordinates, so they work at any SITL `-L` location. Set an absolute
site anyway if you want reproducible runs.

---

## 9. Reference

- ArduPilot SITL: <https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html>
- Setting up SITL on Linux: <https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html>
- Using SITL: <https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html>
- MAVLink connection strings: <https://ardupilot.org/dev/docs/mavlink-connecting.html>
- `sim_vehicle.py` flags: `python3 ~/ardupilot/Tools/autotest/sim_vehicle.py --help`
- Guided mode: <https://ardupilot.org/copter/docs/ac_WP_GPS_modes.html>
