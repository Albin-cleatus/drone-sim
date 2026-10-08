#!/usr/bin/env python3
"""
fake_sitl.py - a deliberately small stand-in for ArduPilot SITL.

It speaks just enough MAVLink for sitl_mission.py to run a whole mission
offline: heartbeats, a simulated GPS/EKF that takes a while to converge, arming
checks that can refuse, mode changes, takeoff, guided position targets with a
3 s expiry (matching ArduPilot's GUID_TIMEOUT), and a draining battery.

Use it to sanity-check the Python side of the setup before building ArduPilot.
It is NOT a flight dynamics model and does not fly like a quadcopter - it
translates at a fixed rate towards whatever setpoint it was last given.

    python tools/fake_sitl.py --out 127.0.0.1:14550 --bind 0.0.0.0:14551
    python sitl_mission.py --connect udpin:0.0.0.0:14550
"""

from __future__ import annotations

import argparse
import math
import socket
import sys
import time
from datetime import datetime

try:
    from pymavlink import mavutil
except ImportError:
    sys.stderr.write("ERROR: pymavlink is not installed.\n"
                     "  python -m pip install -r requirements.txt\n")
    raise SystemExit(2)

ML = mavutil.mavlink

# ArduPilot's default SITL location (CMAC, Canberra).
HOME_LAT = -35.363261
HOME_LON = 149.165230
HOME_ALT_MSL_M = 584.0

COPTER_MODES = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4,
    "LOITER": 5, "RTL": 6, "CIRCLE": 7, "LAND": 9, "DRIFT": 11, "SPORT": 13,
    "FLIP": 14, "AUTOTUNE": 15, "POSHOLD": 16, "BRAKE": 17, "THROW": 18,
    "AVOID_ADSB": 19, "GUIDED_NOGPS": 20, "SMART_RTL": 21, "FLOWHOLD": 22,
    "FOLLOW": 23, "ZIGZAG": 24, "SYSTEMID": 25, "AUTOROTATE": 26,
    "AUTO_RTL": 27,
}
MODE_NAMES = {v: k for k, v in COPTER_MODES.items()}

GUID_TIMEOUT = 3.0          # ArduPilot GUID_TIMEOUT default, seconds
DISARM_DELAY = 3.0          # stand-in for ArduPilot's DISARM_DELAY, seconds
CLIMB_RATE = 2.5            # m/s
CRUISE_RATE = 5.0           # m/s
TICK = 0.02                 # 50 Hz


def log(msg: str) -> None:
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


class UdpWriter:
    """Minimal file-like shim so pymavlink can write frames onto a UDP socket."""

    def __init__(self, sock, addr):
        self.sock = sock
        self.addr = addr

    def write(self, buf):
        try:
            self.sock.sendto(buf, self.addr)
        except OSError:
            pass


class FakeSitl:
    def __init__(self, args):
        self.args = args
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((args.bind_host, args.bind_port))
        self.sock.setblocking(False)

        host, port = args.out.rsplit(":", 1)
        self.out = (host, int(port))
        self.writer = UdpWriter(self.sock, self.out)
        self.mav = ML.MAVLink(self.writer, srcSystem=1, srcComponent=1)
        self.mav.robust_parsing = True

        self.t0 = time.monotonic()
        self.lat = HOME_LAT
        self.lon = HOME_LON
        self.alt = 0.0                     # relative to home, metres
        self.armed = False
        self.mode = COPTER_MODES["STABILIZE"]
        self.target_alt = None
        self.target_lat = None
        self.target_lon = None
        self.target_stamp = 0.0
        self.deny_arms = args.deny_arms
        self.params = {"ARMING_CHECK": 1, "GUID_TIMEOUT": GUID_TIMEOUT}
        self.batt_mah_used = 0.0
        self.current_a = 0.0
        self._land_since = None
        self._seen_masks = set()
        self._next = {}

    # -- simulation --------------------------------------------------------- #

    def gps_fix(self) -> int:
        el = time.monotonic() - self.t0
        if el < self.args.lock_seconds:
            return 0 if el < self.args.lock_seconds * 0.6 else 2
        return 3

    def gps_sats(self) -> int:
        el = time.monotonic() - self.t0
        if el >= self.args.lock_seconds:
            return 12
        return int(12 * el / self.args.lock_seconds)

    def step(self, dt: float):
        now = time.monotonic()
        in_end_mode = self.mode in (COPTER_MODES["RTL"], COPTER_MODES["LAND"])

        # Vertical, while following a takeoff/guided target.
        if not in_end_mode and self.target_alt is not None:
            err = self.target_alt - self.alt
            step = max(-CLIMB_RATE * dt, min(CLIMB_RATE * dt, err))
            self.alt += step
            if abs(err) < 0.02:
                self.alt = self.target_alt

        # Guided targets expire, exactly as ArduPilot's GUID_TIMEOUT makes them.
        if self.target_lat is not None and now - self.target_stamp > GUID_TIMEOUT:
            self.target_lat = self.target_lon = None

        # Horizontal, while following a guided setpoint.
        if self.armed and self.target_lat is not None and not in_end_mode:
            dn = (self.target_lat - self.lat) * 111320.0
            de = (self.target_lon - self.lon) * 111320.0 * math.cos(math.radians(self.lat))
            dist = math.hypot(dn, de)
            if dist > 1e-6:
                move = min(CRUISE_RATE * dt, dist)
                self.lat += (dn / dist) * move / 111320.0
                self.lon += (de / dist) * move / (111320.0 * math.cos(math.radians(self.lat)))

        # RTL / LAND: come home, descend, then auto-disarm after a delay.
        if self.armed and in_end_mode:
            if self.mode == COPTER_MODES["RTL"]:
                dn = (HOME_LAT - self.lat) * 111320.0
                de = (HOME_LON - self.lon) * 111320.0 * math.cos(math.radians(self.lat))
                dist = math.hypot(dn, de)
                if dist > 1.0:
                    move = min(CRUISE_RATE * dt, dist)
                    self.lat += (dn / dist) * move / 111320.0
                    self.lon += (de / dist) * move / (111320.0 * math.cos(math.radians(self.lat)))
                    self._land_since = None
                    self._settle(dt, 0.0)
                    self._battery_step(dt)
                    return
            self._settle(dt, CLIMB_RATE)

        if self.alt <= 0.01 and self.armed and in_end_mode:
            if self._land_since is None:
                self._land_since = now
                log("landed, waiting for auto-disarm")
            elif now - self._land_since >= DISARM_DELAY:
                self.armed = False
                self.target_alt = self.target_lat = self.target_lon = None
                log("auto-DISARMED")
        elif not in_end_mode:
            self._land_since = None

        self._battery_step(dt)

    def _settle(self, dt: float, rate: float):
        """Descend at `rate`, or hold altitude when rate is 0."""
        if rate <= 0.0:
            return
        self.alt = max(0.0, self.alt - rate * dt)

    def _battery_step(self, dt: float):
        amps = 18.0 if (self.armed and self.alt > 0.5) else (1.0 if self.armed else 0.5)
        self.batt_mah_used += amps * 1000.0 * dt / 3600.0
        self.current_a = amps

    def battery(self):
        frac = max(0.0, 1.0 - self.batt_mah_used / self.args.batt_mah)
        # 4S: 16.8 V full, 13.2 V empty.
        volts = 13.2 + 3.6 * frac
        return volts, frac

    # -- outbound messages -------------------------------------------------- #

    def _due(self, key: str, hz: float) -> bool:
        now = time.monotonic()
        if now >= self._next.get(key, 0.0):
            self._next[key] = now + 1.0 / hz
            return True
        return False

    def heartbeat(self):
        base = ML.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.armed:
            base |= ML.MAV_MODE_FLAG_SAFETY_ARMED
        # signature: (type, autopilot, base_mode, custom_mode, system_status, mavlink_version=3)
        self.mav.heartbeat_send(
            ML.MAV_TYPE_QUADROTOR, ML.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base, self.mode, ML.MAV_STATE_ACTIVE,
        )

    def send_all(self):
        if self._due("hb", 1.0):
            self.heartbeat()

        if self._due("att", 10.0):
            self.mav.attitude_send(
                int((time.monotonic() - self.t0) * 1000) & 0xFFFFFFFF,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            )

        if self._due("gpi", 10.0):
            self.mav.global_position_int_send(
                int((time.monotonic() - self.t0) * 1000) & 0xFFFFFFFF,
                int(self.lat * 1e7), int(self.lon * 1e7),
                int((HOME_ALT_MSL_M + self.alt) * 1000), int(self.alt * 1000),
                0, 0, 0, 0,
            )

        if self._due("gps", 5.0):
            # signature: (time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible)
            self.mav.gps_raw_int_send(
                int((time.monotonic() - self.t0) * 1e6),
                self.gps_fix(),
                int(self.lat * 1e7), int(self.lon * 1e7),
                int((HOME_ALT_MSL_M + self.alt) * 1000),
                120, 130,              # eph, epv (cm)
                0, 0,                  # vel (cm/s), cog (cdeg)
                self.gps_sats(),
            )

        if self._due("sys", 5.0):
            volts, frac = self.battery()
            self.mav.sys_status_send(
                0, 0, 0,                                     # sensors present/enabled/health
                int(self.sim_load() * 10),                   # load, per-mille
                int(volts * 1000),                           # voltage, mV
                int(self.current_a * 100),                   # current, cA
                int(frac * 100),                             # remaining, %
                0, 0,                                        # drop_rate_comm, errors_comm
                0, 0, 0, 0,                                  # errors_count1..4
            )

        if self._due("batt", 2.0):
            volts, frac = self.battery()
            cells = [int(volts / 4.0 * 1000)] * 4 + [65535] * 6
            # signature: (id, battery_function, type, temperature, voltages,
            #             current_battery, current_consumed, energy_consumed,
            #             battery_remaining)
            self.mav.battery_status_send(
                0, ML.MAV_BATTERY_FUNCTION_ALL, ML.MAV_BATTERY_TYPE_LIPO,
                2500,                                        # temperature, cdegC (25.00 C)
                cells,
                int(self.current_a * 100),                   # current, cA
                int(self.batt_mah_used),                     # consumed, mAh
                -1,                                          # energy consumed, unknown
                int(frac * 100),
            )

        if self._due("hud", 5.0):
            gs = CRUISE_RATE if (self.armed and self.target_lat is not None) else 0.0
            self.mav.vfr_hud_send(
                gs, gs, 0, int(50 if self.alt > 0.5 else 0), self.alt, 0.0,
            )

        if self._due("ekf", 2.0):
            flags = 0
            if self.gps_fix() >= 3:
                flags = (ML.EKF_ATTITUDE | ML.EKF_VELOCITY_HORIZ | ML.EKF_VELOCITY_VERT
                         | ML.EKF_POS_VERT_ABS | ML.EKF_POS_HORIZ_ABS)
            # signature: (flags, velocity_variance, pos_horiz_variance,
            #             pos_vert_variance, compass_variance, terrain_alt_variance)
            self.mav.ekf_status_report_send(flags, 0.1, 0.1, 0.1, 0.0, 0.0)

        if self._due("pwr", 1.0):
            self.mav.power_status_send(int(5.0 * 1000), int(5.0 * 1000), 0)

    def sim_load(self) -> float:
        """Pretend the simulated scheduler is a bit busier while flying."""
        return 22.0 + (8.0 if self.armed else 0.0)

    # -- inbound handling --------------------------------------------------- #

    def ack(self, command: int, result: int):
        self.mav.command_ack_send(command, result)

    def statustext(self, text: str, severity: int = 4):
        self.mav.statustext_send(severity, text.encode("utf-8"))

    def handle(self, msg):
        t = msg.get_type()
        if t == "COMMAND_LONG":
            self.on_command_long(msg)
        elif t == "SET_POSITION_TARGET_GLOBAL_INT":
            self.on_setpoint(msg)
        elif t == "PARAM_SET":
            self.on_param_set(msg)
        elif t == "PARAM_REQUEST_LIST":
            for name, value in self.params.items():
                self.mav.param_value_send(
                    name.encode("utf-8"), float(value),
                    ML.MAV_PARAM_TYPE_REAL32, len(self.params), 0,
                )

    def on_command_long(self, msg):
        cmd = msg.command
        p = [msg.param1, msg.param2, msg.param3, msg.param4, msg.param5, msg.param6, msg.param7]

        if cmd == ML.MAV_CMD_DO_SET_MODE:
            self.mode = int(p[1])
            self._land_since = None
            log(f"mode -> {MODE_NAMES.get(self.mode, self.mode)}")
            self.ack(cmd, ML.MAV_RESULT_ACCEPTED)

        elif cmd == ML.MAV_CMD_COMPONENT_ARM_DISARM:
            if p[0] >= 0.5:
                if self.deny_arms > 0:
                    self.deny_arms -= 1
                    log(f"arm DENIED ({self.deny_arms} more denials queued)")
                    self.statustext("PreArm: simulated denial", ML.MAV_SEVERITY_CRITICAL)
                    self.ack(cmd, ML.MAV_RESULT_DENIED)
                    return
                if self.gps_fix() < 3:
                    self.statustext("PreArm: GPS: Bad fix", ML.MAV_SEVERITY_CRITICAL)
                    self.ack(cmd, ML.MAV_RESULT_DENIED)
                    return
                self.armed = True
                log("ARMED")
            else:
                self.armed = False
                self.target_alt = None
                self.target_lat = self.target_lon = None
                log("DISARMED")
            self.ack(cmd, ML.MAV_RESULT_ACCEPTED)

        elif cmd == ML.MAV_CMD_NAV_TAKEOFF:
            if not self.armed:
                self.ack(cmd, ML.MAV_RESULT_DENIED)
                return
            self.target_alt = p[6]
            log(f"takeoff -> {self.target_alt:.1f} m")
            self.ack(cmd, ML.MAV_RESULT_ACCEPTED)

        elif cmd == ML.MAV_CMD_SET_MESSAGE_INTERVAL:
            self.ack(cmd, ML.MAV_RESULT_ACCEPTED)

        else:
            self.ack(cmd, ML.MAV_RESULT_UNSUPPORTED)

    def on_setpoint(self, msg):
        frame = msg.coordinate_frame
        if frame != ML.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            log(f"setpoint in unexpected frame {frame}, ignoring")
            return
        if (msg.type_mask, round(msg.yaw, 2)) not in self._seen_masks:
            self._seen_masks.add((msg.type_mask, round(msg.yaw, 2)))
            log(f"setpoint type_mask=0x{msg.type_mask:04x} yaw={msg.yaw:+.3f} rad")
        self.target_lat = msg.lat_int / 1e7
        self.target_lon = msg.lon_int / 1e7
        self.target_stamp = time.monotonic()
        if msg.alt is not None:
            self.target_alt = float(msg.alt)

    def on_param_set(self, msg):
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode("utf-8", "replace")
        name = name.rstrip("\x00")
        self.params[name] = msg.param_value
        log(f"param {name} = {msg.param_value:g}")
        self.mav.param_value_send(
            name.encode("utf-8"), float(msg.param_value),
            ML.MAV_PARAM_TYPE_REAL32, len(self.params), 0,
        )

    # -- main loop ---------------------------------------------------------- #

    def run(self):
        log(f"fake SITL listening on {self.args.bind_host}:{self.args.bind_port}, "
            f"sending to {self.out[0]}:{self.out[1]}")
        log(f"GPS locks after {self.args.lock_seconds:.0f}s; "
            f"guided targets expire after {GUID_TIMEOUT:.1f}s")
        last = time.monotonic()
        while True:
            now = time.monotonic()
            dt = min(now - last, 0.1)
            last = now

            while True:
                try:
                    data, _addr = self.sock.recvfrom(4096)
                except BlockingIOError:
                    break
                except OSError:
                    break
                msgs = self.mav.parse_buffer(data)
                for m in msgs or []:
                    if m.get_type() != "BAD_DATA":
                        self.handle(m)

            self.step(dt)
            self.send_all()
            time.sleep(TICK)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Minimal fake ArduPilot SITL for offline testing.")
    p.add_argument("--bind", default="0.0.0.0:14551",
                   help="host:port to receive MAVLink on")
    p.add_argument("--out", default="127.0.0.1:14550",
                   help="host:port to send the MAVLink stream to")
    p.add_argument("--lock-seconds", type=float, default=10.0,
                   help="how long the simulated GPS takes to reach a 3D fix")
    p.add_argument("--deny-arms", type=int, default=0,
                   help="refuse this many arm attempts (to exercise retry logic)")
    p.add_argument("--batt-mah", type=float, default=5000.0,
                   help="simulated pack capacity, mAh")
    args = p.parse_args(argv)
    args.bind_host, port = args.bind.rsplit(":", 1)
    args.bind_port = int(port)
    return args


if __name__ == "__main__":
    try:
        FakeSitl(parse_args()).run()
    except KeyboardInterrupt:
        log("stopped")
