#!/usr/bin/env python3
"""
sitl_mission.py - drive an ArduPilot SITL quadcopter through a short autonomous
mission and log telemetry, autopilot CPU load, MAVLink packet rate and battery
state to a local CSV file.

Mission
-------
  1. Connect over UDP and wait for a usable GPS fix / EKF solution.
  2. Switch to GUIDED, arm, take off to --takeoff-alt metres.
  3. Fly three waypoints (metres north/east of the takeoff point) using
     SET_POSITION_TARGET_GLOBAL_INT.
  4. Hold LOITER, then finish with RTL (or LAND / nothing).

Design notes
------------
* The whole script is single threaded. Every wait pumps the MAVLink receive
  loop, so CSV logging and packet accounting keep running while the script is
  blocked waiting for the vehicle. This also keeps message counting exact.
* Guided targets are re-sent at 2 Hz. ArduPilot expires a guided target after
  GUID_TIMEOUT (default 3 s), so a fire-and-forget target makes the copter stop
  dead in the air.
* "CPU load" is the autopilot's own SYS_STATUS.load figure. In SITL that is
  produced by the simulator, not by real flight hardware, and is often close to
  constant. Pass --sitl-pid to additionally sample the SITL process on the host
  with psutil, which gives a real number you can act on.

Requires: pymavlink.  Optional: psutil (only for --sitl-pid).
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys
import time
from collections import deque
from datetime import datetime, timezone

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - dependency check
    sys.stderr.write(
        "ERROR: pymavlink is not installed.\n"
        "  python -m pip install -r requirements.txt\n"
    )
    raise SystemExit(2)


# --------------------------------------------------------------------------- #
# constants
# --------------------------------------------------------------------------- #

EARTH_R = 6371000.0

# ArduCopter custom mode numbers. Used as a fallback if the vehicle does not
# hand us a mode map (we normally read the map straight from pymavlink).
COPTER_MODES = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4,
    "LOITER": 5, "RTL": 6, "CIRCLE": 7, "LAND": 9, "DRIFT": 11, "SPORT": 13,
    "FLIP": 14, "AUTOTUNE": 15, "POSHOLD": 16, "BRAKE": 17, "THROW": 18,
    "AVOID_ADSB": 19, "GUIDED_NOGPS": 20, "SMART_RTL": 21, "FLOWHOLD": 22,
    "FOLLOW": 23, "ZIGZAG": 24, "SYSTEMID": 25, "AUTOROTATE": 26,
    "AUTO_RTL": 27,
}

# SET_POSITION_TARGET_GLOBAL_INT type_mask bits (MAVLink spec, 2**bit).
_IGNORE_VX, _IGNORE_VY, _IGNORE_VZ = 1 << 3, 1 << 4, 1 << 5
_IGNORE_AX, _IGNORE_AY, _IGNORE_AZ = 1 << 6, 1 << 7, 1 << 8
_IGNORE_YAW, _IGNORE_YAW_RATE = 1 << 10, 1 << 11

# EKF_STATUS_REPORT flag bits, with local fallbacks in case the installed
# pymavlink predates the named constants.
EKF_ATTITUDE = getattr(mavutil.mavlink, "EKF_ATTITUDE", 1 << 0)
EKF_VELOCITY_HORIZ = getattr(mavutil.mavlink, "EKF_VELOCITY_HORIZ", 1 << 1)
EKF_VELOCITY_VERT = getattr(mavutil.mavlink, "EKF_VELOCITY_VERT", 1 << 2)
EKF_POS_HORIZ_ABS = getattr(mavutil.mavlink, "EKF_POS_HORIZ_ABS", 1 << 4)
EKF_POS_VERT_ABS = getattr(mavutil.mavlink, "EKF_POS_VERT_ABS", 1 << 5)

CSV_FIELDS = [
    "timestamp_utc", "t_rel_s", "link", "mode", "armed", "wp_index", "wp_dist_m",
    "lat", "lon", "alt_rel_m", "alt_msl_m",
    "vn_mps", "ve_mps", "vd_mps", "groundspeed_mps",
    "roll_deg", "pitch_deg", "yaw_deg", "heading_deg", "throttle_pct", "climb_mps",
    "gps_fix", "gps_sats", "gps_eph", "ekf_ok",
    "cpu_load_pct", "host_cpu_pct", "rx_rate_hz", "rx_kbps",
    "sys_drop_rate_pct", "sys_errors_comm",
    "batt_voltage_v", "batt_voltage_pack_v", "batt_current_a",
    "batt_remaining_pct", "batt_temp_c",
]

# Messages worth pulling at a known rate. ArduPilot honours SET_MESSAGE_INTERVAL
# for most of these; anything it refuses is simply skipped.
STREAM_REQUEST = {
    "ATTITUDE": 10, "GLOBAL_POSITION_INT": 10, "GPS_RAW_INT": 5,
    "SYS_STATUS": 5, "BATTERY_STATUS": 2, "VFR_HUD": 5,
    "EKF_STATUS_REPORT": 2, "POWER_STATUS": 1,
}

# COMMAND_ACK results that mean "this will not work, stop waiting".
REJECT_RESULTS = frozenset(
    getattr(mavutil.mavlink, name) for name in (
        "MAV_RESULT_TEMPORARILY_REJECTED", "MAV_RESULT_DENIED",
        "MAV_RESULT_UNSUPPORTED", "MAV_RESULT_FAILED",
    ) if hasattr(mavutil.mavlink, name)
)

# Stick positions used to hold altitude during the LOITER hold.
#
# LOITER is a pilot-controlled mode: ModeLoiter::run() takes its climb rate from
# get_pilot_desired_climb_rate_ms(), i.e. the throttle stick, and - unlike GUIDED -
# it has no GCS altitude target to fall back on. Headless SITL has no RC
# transmitter, so throttle reads at minimum, which is below the THR_DZ (default
# 100) deadband and commands the full descent rate PILOT_SPD_UP (2.5 m/s). The
# result is that a "hold" descends to the ground and the RTL afterwards is a
# no-op on an already-landed vehicle.
#
# 1500 us is mid-stick for the default RC3_MIN/RC3_MAX/RC3_TRIM of
# 1000/2000/1500: get_throttle_mid() returns control-in 500, the deadband spans
# 400..600, so this asks for exactly zero climb rate. Roll, pitch and yaw are
# centred too, so the hold holds instead of drifting.
RC_OVERRIDE_HOLD = (1500, 1500, 1500, 1500, 0, 0, 0, 0)

# All-zero channels release an override. ArduPilot also expires one by itself
# after RC_OVERRIDE_TIME (default 3 s), which is why the hold re-sends at 2 Hz.
RC_OVERRIDE_RELEASE = (0, 0, 0, 0, 0, 0, 0, 0)


class MissionError(RuntimeError):
    """Raised when the mission cannot proceed."""


# --------------------------------------------------------------------------- #
# small helpers
# --------------------------------------------------------------------------- #

def log(msg: str, level: str = "info") -> None:
    prefix = {"info": "   ", "step": "== ", "warn": " ! ", "err": "XX "}.get(level, "   ")
    clock = datetime.now().strftime("%H:%M:%S")
    print(f"[{clock}] {prefix}{msg}", flush=True)


def offset_latlon(lat_deg: float, lon_deg: float, north_m: float, east_m: float):
    """Flat-earth offset, plenty accurate over the few tens of metres we fly."""
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * math.cos(math.radians(lat_deg)))
    return lat_deg + dlat, lon_deg + dlon


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def fmt(v, nd: int = 2) -> str:
    """CSV cell formatter: blank for missing values instead of 'nan'."""
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        if math.isnan(v):
            return ""
        return f"{v:.{nd}f}"
    return str(v)


def parse_waypoints(text: str):
    """'25,0;25,25;0,25' -> [(25.0, 0.0), (25.0, 25.0), (0.0, 25.0)] as (N, E)."""
    wps = []
    for chunk in re.split(r"[;\s]+", text.strip()):
        if not chunk:
            continue
        parts = chunk.split(",")
        if len(parts) != 2:
            raise argparse.ArgumentTypeError(
                f"bad waypoint {chunk!r}; expected 'north_m,east_m'"
            )
        try:
            wps.append((float(parts[0]), float(parts[1])))
        except ValueError:
            raise argparse.ArgumentTypeError(f"bad waypoint {chunk!r}: not numeric")
    if not wps:
        raise argparse.ArgumentTypeError("no waypoints given")
    return wps


def pos_type_mask(use_yaw: bool) -> int:
    """Position-only type_mask, optionally leaving yaw under our control."""
    mask = _IGNORE_VX | _IGNORE_VY | _IGNORE_VZ | _IGNORE_AX | _IGNORE_AY | _IGNORE_AZ
    mask |= _IGNORE_YAW_RATE | (_IGNORE_YAW if not use_yaw else 0)
    return mask


# --------------------------------------------------------------------------- #
# vehicle state
# --------------------------------------------------------------------------- #

class Telemetry:
    """Latest value of everything we log. NaN means 'not seen yet'."""

    def __init__(self):
        self.mode = ""
        self.armed = False
        self.lat = math.nan
        self.lon = math.nan
        self.alt_rel = math.nan
        self.alt_msl = math.nan
        self.vn = math.nan
        self.ve = math.nan
        self.vd = math.nan
        self.groundspeed = math.nan
        self.roll = math.nan
        self.pitch = math.nan
        self.yaw = math.nan
        self.heading = math.nan
        self.throttle = math.nan
        self.climb = math.nan
        self.gps_fix = 0
        self.gps_sats = 0
        self.gps_eph = math.nan
        self.ekf_ok = False
        self.ekf_seen = False
        self.cpu_load = math.nan
        self.host_cpu = math.nan
        self.sys_drop_rate = math.nan
        self.sys_errors_comm = math.nan
        self.batt_voltage = math.nan
        self.batt_voltage_pack = math.nan
        self.batt_current = math.nan
        self.batt_remaining = math.nan
        self.batt_temp = math.nan
        self.rx_rate_hz = 0.0
        self.rx_kbps = 0.0
        self.wp_dist = math.nan


# --------------------------------------------------------------------------- #
# the vehicle
# --------------------------------------------------------------------------- #

class SitlVehicle:
    def __init__(self, args):
        self.args = args
        self.master = None
        self.tel = Telemetry()
        self.mode_map = dict(COPTER_MODES)

        self.t0 = time.monotonic()
        self._last_log = 0.0
        self._window_start = self.t0
        self._window_msgs = 0
        self._window_bytes = 0

        self.msg_count = 0
        self.byte_count = 0
        self._last_rx = time.monotonic()
        self._link_warned = False

        self.statustexts = deque(maxlen=8)
        self._param_echo = {}
        self._acks = {}

        self._csv_file = None
        self._csv = None
        self._rows = 0

        self.home = (math.nan, math.nan)
        self.waypoints = []          # list of (lat, lon, alt_rel)
        self.wp_index = 0

        self._sitl_proc = None

    # -- targets ------------------------------------------------------------ #

    def _targets(self):
        return self.master.target_system, self.master.target_component

    def _boot_ms(self) -> int:
        return int((time.monotonic() - self.t0) * 1000) & 0xFFFFFFFF

    # -- lifecycle ---------------------------------------------------------- #

    def connect(self):
        log(f"connecting to {self.args.connect}", "step")
        try:
            self.master = mavutil.mavlink_connection(self.args.connect)
        except OSError as exc:
            raise MissionError(
                f"could not open {self.args.connect}: {exc}\n"
                "     Is another GCS already bound to that port? "
                "sim_vehicle.py --out=udp:... does not bind it, but a MAVProxy "
                "started with --master=udp:... would."
            )
        # Tolerate malformed packets instead of blowing up mid-mission.
        self.master.robust_parsing = True

        # Pump our own loop rather than mavutil.wait_heartbeat() so that packet
        # accounting starts from the very first byte.
        deadline = time.monotonic() + self.args.connect_timeout
        while time.monotonic() < deadline:
            self._drain()
            if self.master.target_system:
                break
            time.sleep(0.02)
        else:
            raise MissionError(
                f"no autopilot heartbeat within {self.args.connect_timeout:.0f}s. "
                "Is SITL running and is --out pointing at this host/port?"
            )

        log(f"vehicle sysid={self.master.target_system} "
            f"compid={self.master.target_component} type={self.master.mav_type}")
        if not self.args.no_stream_setup:
            self._configure_streams()

    def _configure_streams(self):
        """Best effort: ask for the messages we log at a sane rate."""
        sent = []
        for name, hz in STREAM_REQUEST.items():
            msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}", None)
            if msg_id is None:
                continue
            ts, tc = self._targets()
            try:
                self.master.mav.command_long_send(
                    ts, tc,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, int(1e6 / hz), 0, 0, 0, 0, 0,
                )
                sent.append(name)
            except Exception:
                pass
        if sent:
            log(f"requested stream intervals: {', '.join(sent)}")
        self._pump_for(0.5)

    def close(self):
        if self._csv_file:
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except Exception:
                pass
            log(f"wrote {self._rows} telemetry rows to {self.args.log_csv}")
            self._csv_file = None

    def abort(self):
        """Best effort tidy-up after Ctrl+C or a failure."""
        if self.master is None:
            return
        if self.tel.armed:
            try:
                log("aborting: switching to RTL", "warn")
                self._send_mode(self.mode_map.get("RTL", 6))
                self._pump_for(1.0)
            except Exception:
                pass

    # -- CSV ---------------------------------------------------------------- #

    def open_csv(self):
        path = self.args.log_csv
        if path is None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join("logs", f"telemetry_{stamp}.csv")
        self.args.log_csv = path
        parent = os.path.dirname(os.path.abspath(path))
        os.makedirs(parent, exist_ok=True)
        self._csv_file = open(path, "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(CSV_FIELDS)
        self._csv_file.flush()
        log(f"logging telemetry to {path}")

    def _row(self):
        tel = self.tel
        now = time.monotonic()
        row = {
            "timestamp_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z",
            "t_rel_s": fmt(now - self.t0, 3),
            "link": "up" if self.link_alive() else "STALE",
            "mode": tel.mode,
            "armed": tel.armed,
            "wp_index": self.wp_index,
            "wp_dist_m": fmt(tel.wp_dist, 2),
            "lat": fmt(tel.lat, 7),
            "lon": fmt(tel.lon, 7),
            "alt_rel_m": fmt(tel.alt_rel, 2),
            "alt_msl_m": fmt(tel.alt_msl, 2),
            "vn_mps": fmt(tel.vn, 2),
            "ve_mps": fmt(tel.ve, 2),
            "vd_mps": fmt(tel.vd, 2),
            "groundspeed_mps": fmt(tel.groundspeed, 2),
            "roll_deg": fmt(tel.roll, 2),
            "pitch_deg": fmt(tel.pitch, 2),
            "yaw_deg": fmt(tel.yaw, 1),
            "heading_deg": fmt(tel.heading, 1),
            "throttle_pct": fmt(tel.throttle, 1),
            "climb_mps": fmt(tel.climb, 2),
            "gps_fix": tel.gps_fix,
            "gps_sats": tel.gps_sats,
            "gps_eph": fmt(tel.gps_eph, 2),
            "ekf_ok": tel.ekf_ok,
            "cpu_load_pct": fmt(tel.cpu_load, 1),
            "host_cpu_pct": fmt(tel.host_cpu, 1),
            "rx_rate_hz": fmt(tel.rx_rate_hz, 1),
            "rx_kbps": fmt(tel.rx_kbps, 2),
            "sys_drop_rate_pct": fmt(tel.sys_drop_rate, 2),
            "sys_errors_comm": fmt(tel.sys_errors_comm, 0),
            "batt_voltage_v": fmt(tel.batt_voltage, 3),
            "batt_voltage_pack_v": fmt(tel.batt_voltage_pack, 3),
            "batt_current_a": fmt(tel.batt_current, 2),
            "batt_remaining_pct": fmt(tel.batt_remaining, 0),
            "batt_temp_c": fmt(tel.batt_temp, 1),
        }
        return [row[f] for f in CSV_FIELDS]

    def _maybe_log(self, now: float):
        if self._csv is None:
            return
        if now - self._last_log < 1.0 / self.args.log_hz:
            return
        self._last_log = now
        self._sample_host_cpu()
        self._csv.writerow(self._row())
        self._rows += 1
        if self._rows % 25 == 0:
            self._csv_file.flush()

    def _sample_host_cpu(self):
        if self._sitl_proc is None:
            return
        try:
            value = self._sitl_proc.cpu_percent(None)
        except Exception:
            value = None
        if value is None or not math.isfinite(value):
            log(f"host CPU sampling of pid {self.args.sitl_pid} gave no usable "
                "value; disabling it", "warn")
            self._sitl_proc = None
            self.tel.host_cpu = math.nan
            return
        self.tel.host_cpu = value

    def init_host_cpu(self):
        if self.args.sitl_pid is None:
            return
        if self.args.sitl_pid < 1:
            log(f"--sitl-pid {self.args.sitl_pid} is not a real process id; "
                "host CPU sampling disabled", "warn")
            return
        try:
            import psutil
        except ImportError:
            log("psutil not installed; --sitl-pid ignored", "warn")
            return
        try:
            self._sitl_proc = psutil.Process(self.args.sitl_pid)
            self._sitl_proc.cpu_percent(None)     # prime the delta
            log(f"sampling host CPU of pid {self.args.sitl_pid} "
                f"({self._sitl_proc.name()})")
        except Exception as exc:
            log(f"cannot attach to pid {self.args.sitl_pid}: {exc}", "warn")
            self._sitl_proc = None

    # -- receive loop ------------------------------------------------------- #

    def link_alive(self) -> bool:
        return (time.monotonic() - self._last_rx) < 5.0

    def _drain(self):
        if self.master is None:
            return
        for _ in range(5000):                      # safety valve per call
            try:
                msg = self.master.recv_match(blocking=False)
            except Exception as exc:
                log(f"link read error: {exc}", "warn")
                return
            if msg is None:
                break
            nbytes = 0
            try:
                nbytes = len(msg.get_msgbuf())
            except Exception:
                pass
            self.msg_count += 1
            self.byte_count += nbytes
            self._window_msgs += 1
            self._window_bytes += nbytes
            self._last_rx = time.monotonic()
            if self._link_warned:
                log("link recovered")
                self._link_warned = False
            self._handle(msg)

        if not self.link_alive() and not self._link_warned and self.msg_count:
            log("no MAVLink traffic for 5 s - link may be down", "warn")
            self._link_warned = True

    def _update_rates(self, now: float):
        dt = now - self._window_start
        if dt >= 1.0:
            self.tel.rx_rate_hz = self._window_msgs / dt
            self.tel.rx_kbps = (self._window_bytes * 8.0 / 1000.0) / dt
            self._window_msgs = 0
            self._window_bytes = 0
            self._window_start = now

    def _pump(self, timeout: float = 0.0, cond=None):
        """
        Run the receive loop. Returns True if `cond` became true, False on
        timeout. `timeout=0` with no condition just services the link once.
        """
        deadline = time.monotonic() + timeout
        while True:
            now = time.monotonic()
            self._drain()
            self._update_rates(now)
            self._maybe_log(now)
            if cond is not None and cond():
                return True
            if now >= deadline:
                return False
            time.sleep(0.005)

    def _pump_for(self, seconds: float):
        self._pump(timeout=seconds)

    def _await(self, cond, timeout: float, desc: str, on_tick=None, tick_hz: float = 1.0):
        """Pump until `cond()` is true; call `on_tick()` at tick_hz while waiting."""
        deadline = time.monotonic() + timeout
        next_tick = 0.0
        while True:
            now = time.monotonic()
            self._drain()
            self._update_rates(now)
            self._maybe_log(now)
            if cond():
                return True
            if now >= deadline:
                log(f"timed out after {timeout:.0f}s waiting for {desc}", "warn")
                return False
            if on_tick is not None and now >= next_tick:
                on_tick()
                next_tick = now + 1.0 / tick_hz
            time.sleep(0.005)

    # -- message handling --------------------------------------------------- #

    def _handle(self, msg):
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            return
        # Dispatch to _on_<message name> if we care about this message.
        handler = getattr(self, f"_on_{mtype.lower()}", None)
        if handler is not None:
            handler(msg)

    def _on_heartbeat(self, msg):
        # Only the autopilot's own heartbeat; MAVProxy/GCS heartbeats on the same
        # link have autopilot == MAV_AUTOPILOT_INVALID.
        if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
            return
        if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return

        if not self.master.target_system:
            self.master.target_system = msg.get_srcSystem()
            self.master.target_component = msg.get_srcComponent()
            # mavutil derives its mode map from mav_type, which it would normally
            # have set in wait_heartbeat(); we bypassed that, so set it here.
            self.master.mav_type = msg.type
            try:
                mapping = self.master.mode_mapping()
                if mapping:
                    self.mode_map = dict(mapping)
            except Exception:
                pass
            self.mode_map.setdefault("GUIDED", 4)
            log(f"mode map: {', '.join(sorted(self.mode_map, key=self.mode_map.get))}")

        self.tel.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.tel.mode = self._mode_name(msg)

    def _mode_name(self, msg) -> str:
        try:
            name = mavutil.mode_string_v10(msg)
            if name:
                return name
        except Exception:
            pass
        reverse = {v: k for k, v in self.mode_map.items()}
        return reverse.get(msg.custom_mode, str(msg.custom_mode))

    def _on_global_position_int(self, msg):
        tel = self.tel
        tel.lat = msg.lat / 1e7
        tel.lon = msg.lon / 1e7
        tel.alt_msl = msg.alt / 1000.0
        tel.alt_rel = msg.relative_alt / 1000.0
        tel.vn = msg.vx / 100.0
        tel.ve = msg.vy / 100.0
        tel.vd = msg.vz / 100.0
        if msg.hdg != 65535:
            tel.heading = msg.hdg / 100.0

    def _on_sys_status(self, msg):
        tel = self.tel
        if msg.load != 65535:
            tel.cpu_load = msg.load / 10.0          # per-mille -> percent
        if msg.voltage_battery not in (0, 65535):
            tel.batt_voltage = msg.voltage_battery / 1000.0
        if msg.current_battery != -1:
            tel.batt_current = msg.current_battery / 100.0
        if msg.battery_remaining != -1:
            tel.batt_remaining = float(msg.battery_remaining)
        tel.sys_drop_rate = msg.drop_rate_comm / 100.0   # centi-percent -> percent
        tel.sys_errors_comm = float(msg.errors_comm)

    def _on_battery_status(self, msg):
        tel = self.tel
        if msg.current_battery != -1:
            tel.batt_current = msg.current_battery / 100.0
        if msg.battery_remaining != -1:
            tel.batt_remaining = float(msg.battery_remaining)
        if msg.temperature not in (0, 32767):
            tel.batt_temp = msg.temperature / 100.0
        cells = [v for v in msg.voltages if v not in (0, 65535)]
        if cells:
            tel.batt_voltage_pack = sum(cells) / 1000.0

    def _on_gps_raw_int(self, msg):
        tel = self.tel
        tel.gps_fix = msg.fix_type
        tel.gps_sats = msg.satellites_visible
        if msg.eph != 65535:
            tel.gps_eph = msg.eph / 100.0

    def _on_attitude(self, msg):
        tel = self.tel
        tel.roll = math.degrees(msg.roll)
        tel.pitch = math.degrees(msg.pitch)
        tel.yaw = math.degrees(msg.yaw) % 360.0

    def _on_vfr_hud(self, msg):
        tel = self.tel
        tel.groundspeed = msg.groundspeed
        tel.throttle = msg.throttle
        tel.climb = msg.climb
        if math.isfinite(msg.heading):
            tel.heading = float(msg.heading)

    def _on_ekf_status_report(self, msg):
        tel = self.tel
        tel.ekf_seen = True
        need = (EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_VELOCITY_VERT
                | EKF_POS_VERT_ABS | EKF_POS_HORIZ_ABS)
        tel.ekf_ok = (msg.flags & need) == need

    def _on_statustext(self, msg):
        text = msg.text
        if isinstance(text, bytes):
            text = text.decode("utf-8", "replace")
        text = text.rstrip("\x00").strip()
        if not text:
            return
        self.statustexts.append(text)
        severity = getattr(msg, "severity", 6)
        level = "warn" if severity <= 4 else "info"
        log(f"vehicle: {text}", level)

    def _on_param_value(self, msg):
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode("utf-8", "replace")
        self._param_echo[name.rstrip("\x00")] = msg.param_value

    def _on_command_ack(self, msg):
        self._acks[msg.command] = msg.result

    # -- commands ----------------------------------------------------------- #

    def _send_mode(self, mode_id: int):
        ts, tc = self._targets()
        self.master.mav.command_long_send(
            ts, tc, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0,
        )

    def set_mode(self, name: str, timeout: float = 15.0) -> bool:
        if name not in self.mode_map:
            raise MissionError(f"vehicle does not offer mode {name!r}; "
                               f"known: {sorted(self.mode_map)}")
        mode_id = self.mode_map[name]
        if self.tel.mode == name:
            log(f"mode already {name}")
            return True
        log(f"mode -> {name} (custom_mode={mode_id})", "step")
        # Retry: the first DO_SET_MODE can land before the vehicle is listening.
        ok = False
        for _ in range(3):
            self._send_mode(mode_id)
            ok = self._await(lambda: self.tel.mode == name, timeout / 3.0,
                             f"mode {name}")
            if ok:
                break
        if not ok:
            raise MissionError(f"could not switch to {name} (still {self.tel.mode!r})")
        return True

    def set_param(self, name: str, value, timeout: float = 5.0) -> bool:
        ts, tc = self._targets()
        self._param_echo.pop(name, None)
        try:
            self.master.param_set_send(name, float(value))
        except TypeError:
            # Older pymavlink on MAVLink1 wants bytes for param_id.
            self.master.mav.param_set_send(
                ts, tc, name.encode("utf-8"), float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
        ok = self._await(lambda: name in self._param_echo, timeout, f"param {name} ack")
        if ok:
            log(f"param {name} = {self._param_echo[name]:g}")
        return ok

    def arm(self, timeout: float = 30.0, attempts: int = 4) -> bool:
        ts, tc = self._targets()
        arm_cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        for attempt in range(1, attempts + 1):
            if self.tel.armed:
                return True
            log(f"arming (attempt {attempt}/{attempts})", "step")
            self.statustexts.clear()
            self._acks.pop(arm_cmd, None)
            self.master.mav.command_long_send(
                ts, tc, arm_cmd, 0, 1, 0, 0, 0, 0, 0, 0,
            )
            # Stop waiting as soon as the vehicle answers. A refusal then costs
            # one round trip instead of the whole per-attempt budget.
            self._await(
                lambda: self.tel.armed or arm_cmd in self._acks,
                timeout / attempts, "arm",
            )
            if self.tel.armed:
                log("armed")
                return True
            result = self._acks.get(arm_cmd)
            if result in REJECT_RESULTS:
                log(f"  vehicle refused arming (MAV_RESULT {result})", "warn")
            for text in list(self.statustexts):
                log(f"  arming said: {text}", "warn")
            self._pump_for(1.0)
        raise MissionError(
            "failed to arm. Usual SITL causes: no 3D GPS fix yet, or a PreArm "
            "check failing. --skip-arming-checks disables the checks "
            "(ARMING_CHECK=0) if you just want the sim to fly."
        )

    def takeoff(self, alt_m: float, timeout: float = 60.0) -> bool:
        ts, tc = self._targets()
        log(f"takeoff -> {alt_m:.1f} m", "step")
        # MAV_CMD_NAV_TAKEOFF param7 is the target altitude (home-relative).
        self.master.mav.command_long_send(
            ts, tc, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt_m,
        )
        reached = self._await(
            lambda: (not math.isnan(self.tel.alt_rel)) and self.tel.alt_rel >= alt_m - 0.4,
            timeout, f"takeoff to {alt_m:.1f} m",
            on_tick=self._report_progress, tick_hz=1.0,
        )
        if not reached:
            raise MissionError(f"did not reach {alt_m:.1f} m (at {self.tel.alt_rel:.1f} m)")
        log(f"airborne at {self.tel.alt_rel:.1f} m")
        return True

    def _guided_target(self, lat, lon, alt, yaw_deg=None):
        """Send one guided setpoint. Must be repeated at < GUID_TIMEOUT."""
        ts, tc = self._targets()
        use_yaw = yaw_deg is not None
        if use_yaw:
            yaw = math.radians(yaw_deg)
        else:
            yaw = 0.0
        self.master.mav.set_position_target_global_int_send(
            self._boot_ms(), ts, tc,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            pos_type_mask(use_yaw),
            int(lat * 1e7), int(lon * 1e7), float(alt),
            0, 0, 0,          # vx, vy, vz     (ignored)
            0, 0, 0,          # afx, afy, afz  (ignored)
            yaw, 0.0,         # yaw, yaw_rate
        )

    def _report_progress(self):
        tel = self.tel
        log(f"alt {tel.alt_rel:5.1f} m  gs {fmt(tel.groundspeed, 1) or '-':>4} m/s  "
            f"wp{self.wp_index} dist {fmt(tel.wp_dist, 1) or '-':>5} m  "
            f"batt {fmt(tel.batt_voltage, 2) or '-'} V  "
            f"rx {tel.rx_rate_hz:.0f} Hz")

    def goto(self, lat, lon, alt, timeout, radius, label, use_yaw=False):
        def dist():
            if math.isnan(self.tel.lat) or math.isnan(self.tel.lon):
                return math.nan
            return haversine_m(self.tel.lat, self.tel.lon, lat, lon)

        def resend():
            yaw = None
            if use_yaw and not math.isnan(self.tel.lat):
                yaw = bearing_deg(self.tel.lat, self.tel.lon, lat, lon)
            self._guided_target(lat, lon, alt, yaw)
            self.tel.wp_dist = dist()

        log(f"{label}: {lat:.7f}, {lon:.7f} @ {alt:.1f} m "
            f"({dist():.1f} m away) - accepting within {radius:.1f} m", "step")

        # Re-send at 2 Hz: ArduPilot drops a guided target it has not heard about
        # for GUID_TIMEOUT seconds (default 3 s) and stops moving.
        reached = self._await(
            lambda: (not math.isnan(dist())) and dist() <= radius,
            timeout, label, on_tick=resend, tick_hz=2.0,
        )
        if not reached:
            raise MissionError(
                f"{label} not reached within {timeout:.0f}s "
                f"({fmt(dist(), 1)} m short, wp_dist={fmt(self.tel.wp_dist, 1)})"
            )
        log(f"{label} reached ({fmt(dist(), 2)} m from target)")
        return True

    # -- mission ------------------------------------------------------------ #

    def _wait_ready(self):
        log(f"waiting for GPS/EKF (up to {self.args.ready_timeout:.0f}s)", "step")
        last_note = 0.0

        def ready():
            tel = self.tel
            gps = tel.gps_fix >= 3 and tel.gps_sats >= 6
            ekf = tel.ekf_ok or not tel.ekf_seen    # tolerate no EKF_STATUS stream
            return gps and ekf

        def note():
            nonlocal last_note
            now = time.monotonic()
            if now - last_note < 5:
                return
            last_note = now
            log(f"  gps fix={self.tel.gps_fix} sats={self.tel.gps_sats} "
                f"ekf={'ok' if self.tel.ekf_ok else ('n/a' if not self.tel.ekf_seen else 'wait')}")

        if not self._await(ready, self.args.ready_timeout, "GPS/EKF ready", on_tick=note, tick_hz=1.0):
            if self.args.skip_arming_checks:
                log("continuing without a full fix because --skip-arming-checks was given", "warn")
            else:
                raise MissionError(
                    "no 3D fix / EKF solution. Give SITL longer, or check that "
                    "the simulated GPS is enabled."
                )
        log(f"ready: fix={self.tel.gps_fix} sats={self.tel.gps_sats}")

    def _capture_home(self):
        if math.isnan(self.tel.lat):
            raise MissionError("no position to build waypoints from")
        self.home = (self.tel.lat, self.tel.lon)
        log(f"home {self.home[0]:.7f}, {self.home[1]:.7f}")

    def _build_waypoints(self, alt):
        self.waypoints = []
        for i, (north, east) in enumerate(self.args.waypoints):
            lat, lon = offset_latlon(self.home[0], self.home[1], north, east)
            self.waypoints.append((lat, lon, alt))
            log(f"  wp{i + 1}: {north:+.0f} m N, {east:+.0f} m E -> {lat:.7f}, {lon:.7f}")

    def fly_waypoints(self):
        if not self.waypoints:
            return
        log(f"flying {len(self.waypoints)} waypoint(s) in GUIDED", "step")
        for i, (lat, lon, alt) in enumerate(self.waypoints):
            # A failsafe or an accidental mode change would silently break the
            # mission here, so check before each leg.
            if self.tel.mode != "GUIDED":
                log(f"mode is {self.tel.mode!r} before wp{i + 1}; re-selecting GUIDED", "warn")
                self.set_mode("GUIDED")
            self.wp_index = i + 1
            self.goto(lat, lon, alt, self.args.wp_timeout, self.args.wp_radius,
                      f"wp{i + 1}", use_yaw=self.args.yaw_to_waypoint)
        self.tel.wp_dist = math.nan
        log("all waypoints reached", "step")

    def _send_rc_override(self, values):
        ts, tc = self._targets()
        self.master.mav.rc_channels_override_send(ts, tc, *values)

    def _hold_loiter(self, seconds: float):
        """Hold position and altitude in LOITER by centring the sticks.

        Without the override the throttle stick sits at minimum and LOITER
        descends at PILOT_SPD_UP until it lands - see RC_OVERRIDE_HOLD.
        """
        deadline = time.monotonic() + seconds
        next_send = 0.0
        while True:
            now = time.monotonic()
            self._drain()
            self._update_rates(now)
            self._maybe_log(now)
            if now >= next_send:
                self._send_rc_override(RC_OVERRIDE_HOLD)
                next_send = now + 0.5
            if now >= deadline:
                break
            time.sleep(0.005)
        # Hand the sticks back before RTL. RTL ignores pilot input anyway, but a
        # stale override would otherwise linger for RC_OVERRIDE_TIME.
        self._send_rc_override(RC_OVERRIDE_RELEASE)

    def finish(self):
        if self.args.loiter_seconds > 0:
            log(f"holding LOITER for {self.args.loiter_seconds:.0f}s", "step")
            self.set_mode("LOITER")
            self._hold_loiter(self.args.loiter_seconds)
            log(f"hold complete: alt {self.tel.alt_rel:.1f} m, "
                f"batt {fmt(self.tel.batt_voltage, 2) or '-'} V "
                f"({fmt(self.tel.batt_remaining, 0) or '-'} %)")

        action = self.args.end_action
        if action == "none":
            log("--end-action none: leaving the vehicle armed in LOITER", "warn")
            return

        label = "RTL" if action == "rtl" else "LAND"
        log(f"{label}: returning", "step")
        self.set_mode(label)
        done = self._await(
            lambda: not self.tel.armed,
            self.args.end_timeout,
            f"{label} + auto-disarm",
            on_tick=self._report_progress, tick_hz=0.25,
        )
        if done:
            log("landed and disarmed")
        else:
            log(f"still armed after {self.args.end_timeout:.0f}s; "
                "issuing an explicit disarm", "warn")
            self.disarm()

    def disarm(self):
        ts, tc = self._targets()
        self.master.mav.command_long_send(
            ts, tc, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0,
        )
        self._await(lambda: not self.tel.armed, 10.0, "disarm")

    def run(self):
        self.open_csv()
        self.init_host_cpu()
        self.t0 = time.monotonic()
        self._window_start = self.t0
        self._last_rx = self.t0
        try:
            self.connect()
            self._wait_ready()
            self._capture_home()
            self._build_waypoints(self.args.takeoff_alt)

            if self.args.skip_arming_checks:
                log("disabling arming checks (ARMING_CHECK=0)", "warn")
                self.set_param("ARMING_CHECK", 0)

            self.set_mode("GUIDED")
            self.arm()
            self.takeoff(self.args.takeoff_alt, self.args.takeoff_timeout)
            self.fly_waypoints()
            self.finish()
        finally:
            self._summary()
            self.close()

    def _summary(self):
        elapsed = time.monotonic() - self.t0
        rate = self.msg_count / elapsed if elapsed > 0 else 0.0
        log("--- summary " + "-" * 46)
        log(f"duration      : {elapsed:.1f} s")
        log(f"mavlink       : {self.msg_count} msgs, "
            f"{self.byte_count / 1024.0:.1f} KiB, {rate:.0f} msgs/s average")
        log(f"waypoints     : {self.wp_index}/{len(self.waypoints)} reached")
        log(f"battery       : {fmt(self.tel.batt_voltage, 2) or '-'} V, "
            f"{fmt(self.tel.batt_current, 2) or '-'} A, "
            f"{fmt(self.tel.batt_remaining, 0) or '-'} %")
        log(f"autopilot cpu : {fmt(self.tel.cpu_load, 1) or '-'} %"
            + (f", host cpu {fmt(self.tel.host_cpu, 1)} %" if self._sitl_proc else ""))
        log(f"csv           : {self.args.log_csv} ({self._rows} rows)")


# --------------------------------------------------------------------------- #
# entry point
# --------------------------------------------------------------------------- #

def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Fly an ArduPilot SITL quadcopter through a 3-waypoint GUIDED "
                    "mission and log telemetry to CSV.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    g = p.add_argument_group("connection")
    g.add_argument("--connect", default="udpin:0.0.0.0:14550",
                   help="pymavlink connection string. udpin: binds the port and "
                        "receives the stream that 'sim_vehicle.py --out=udp:HOST:PORT' "
                        "sends to you.")
    g.add_argument("--connect-timeout", type=float, default=60.0,
                   help="seconds to wait for the first heartbeat")
    g.add_argument("--no-stream-setup", action="store_true",
                   help="skip SET_MESSAGE_INTERVAL requests")

    m = p.add_argument_group("mission")
    m.add_argument("--takeoff-alt", type=float, default=5.0, help="takeoff altitude, m")
    m.add_argument("--waypoints", type=parse_waypoints, default=parse_waypoints("25,0;25,25;0,25"),
                   metavar="N,E;N,E;...",
                   help="waypoints as metres north,east of the takeoff point")
    m.add_argument("--wp-radius", type=float, default=1.5,
                   help="horizontal acceptance radius, m")
    m.add_argument("--wp-timeout", type=float, default=120.0, help="per-waypoint timeout, s")
    m.add_argument("--takeoff-timeout", type=float, default=60.0,
                   help="seconds to wait for the climb to complete")
    m.add_argument("--ready-timeout", type=float, default=180.0,
                   help="seconds to wait for a 3D fix + EKF")
    m.add_argument("--yaw-to-waypoint", action="store_true",
                   help="also command yaw, so the nose points along track")
    m.add_argument("--loiter-seconds", type=float, default=5.0,
                   help="hold LOITER after the last waypoint before ending")
    m.add_argument("--end-action", choices=("rtl", "land", "none"), default="rtl",
                   help="what to do after the loiter hold")
    m.add_argument("--end-timeout", type=float, default=180.0,
                   help="seconds to wait for the end action to land + disarm")
    m.add_argument("--skip-arming-checks", action="store_true",
                   help="set ARMING_CHECK=0 before arming")

    l = p.add_argument_group("logging")
    l.add_argument("--log-csv", default=None,
                   help="CSV path (default logs/telemetry_<UTC timestamp>.csv)")
    l.add_argument("--log-hz", type=float, default=5.0, help="CSV sample rate")
    l.add_argument("--sitl-pid", type=int, default=None,
                   help="SITL process id to sample host CPU for (needs psutil)")

    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.log_hz <= 0:
        log("--log-hz must be > 0", "err")
        return 2
    veh = SitlVehicle(args)
    try:
        veh.run()
    except KeyboardInterrupt:
        log("interrupted", "warn")
        veh.abort()
        veh.close()
        return 130
    except MissionError as exc:
        log(f"mission failed: {exc}", "err")
        veh.abort()
        veh.close()
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
