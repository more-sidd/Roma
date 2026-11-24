"""Local web UI to control and calibrate the SO-101 arm, motor by motor.

  python arm_ui.py                 real arm, adapter auto-detected
  python arm_ui.py --port COM16    real arm on a given port
  python arm_ui.py --sim           simulated arm, no hardware (for trying the UI)

Then open http://127.0.0.1:8101 in a browser. Ctrl+C stops the server; motors
keep whatever torque state they had (use "Torque off all" first to go limp).

Control tab, safety rules built in (see AGENTS.md):
  * Torque-on is refused if bus voltage > 8.4 V.
  * Torque-on sets goal = current position first, so nothing jumps.
  * Torque-on is refused if the joint sits outside the motor's limit window
    (the motor would snap it to the edge). Calibrate first if that happens.
  * Every goal is clamped into the limit window and moves at the UI's speed
    setting (default 20 deg/s) with a soft acceleration ramp.
  * Goals are only accepted for joints whose torque is on.

Calibration tab: the same procedure as so101_calibrate.py / lerobot-calibrate:
  start (torque off, offsets -> 0, limits -> 0..4095) -> set middle
  (Homing_Offset = raw - 2047) -> record ranges by hand -> save (limits written
  to the motors, JSON saved in LeRobot format). Cancel restores the previous
  offsets and limits. Motion commands are blocked while it runs.
"""
import argparse
import json
import math
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from serial.tools import list_ports

from so101_calibrate import HALF_TURN, lerobot_calib_dir
from remote import Remote
from sts_bus import JOINTS, MAX_RES, Bus, BusError, decode, encode

MAX_VOLTS = 8.4
STEPS_PER_DEG = 4096 / 360  # output-shaft encoder; same for the 1:147 motors
CENTER = HALF_TURN          # 2047
ACCEL = 40                  # x100 steps/s^2 -> gentle ramp
SPEED_RANGE_DPS = (2.0, 120.0)
MIN_SPAN = 100              # steps a joint must travel during range recording (~9 deg)
EDGE_TOL = 23               # steps (~2 deg) past the limit window that torque-on still accepts
HTML = Path(__file__).with_name("arm_ui.html")
REPO_CALIB = Path(__file__).with_name("calibration")

# SO-101 URDF design limits in degrees from the calibrated centre (display only)
URDF_DEG = {"gripper": (-10.0, 100.0), "wrist_roll": (-157.2, 162.8), "wrist_flex": (-95.0, 95.0),
            "elbow_flex": (-96.8, 96.8), "shoulder_lift": (-100.0, 100.0),
            "shoulder_pan": (-110.0, 110.0)}


class CmdError(Exception):
    pass


class Arm:
    def __init__(self, bus, label, sim=False):
        self.bus, self.label, self.sim = bus, label, sim
        self.lock = threading.RLock()
        self.speed_dps = 20.0
        self.motors = {i: {"id": i, "name": n, "online": False, "pos": None, "goal": None,
                           "speed": None, "load": None, "volt": None, "temp": None,
                           "torque": False, "lo": 0, "hi": MAX_RES, "offset": None,
                           "design": URDF_DEG[n]} for i, n in JOINTS.items()}
        self.cal = {"phase": "idle", "robot_id": "my_follower", "ids": [], "joints": {}, "saved": None}
        self._cal_orig = {}
        self.last_poll, self.poll_hz, self.poll_error = 0.0, 0.0, None
        self.remote = None  # set by main() once the Arm exists
        threading.Thread(target=self._poll_loop, daemon=True).start()

    # ---------- polling ----------
    def _poll_loop(self):
        """Never dies: an unexpected error (e.g. the USB adapter hiccups) is reported
        in the state and the loop keeps going, so the page can't silently freeze."""
        while True:
            try:
                for i in JOINTS:
                    self._poll(i)
                self.poll_error = None
            except Exception as e:  # noqa: BLE001 - keep the loop alive whatever happens
                self.poll_error = f"{type(e).__name__}: {e}"
                print("poll error:", self.poll_error)
                time.sleep(0.5)
            now = time.time()
            if self.last_poll:  # full cycle time, including the pause below
                hz = 1 / max(now - self.last_poll, 1e-3)
                self.poll_hz = hz if not self.poll_hz else 0.8 * self.poll_hz + 0.2 * hz
            self.last_poll = now
            time.sleep(0.03)

    def _poll(self, i):
        m = self.motors[i]
        try:
            with self.lock:
                if not m["online"]:
                    if not self.bus.ping(i, retries=1):
                        return
                    self._read_calib_regs(i)
                    m["online"] = True
                d = self.bus.read_raw(i, 56, 8, retries=2)  # pos, speed, load, volt, temp
                torque = self.bus.read(i, "Torque_Enable", retries=2)
            m["pos"] = decode("Present_Position", d[0] | d[1] << 8)
            m["speed"] = decode("Present_Speed", d[2] | d[3] << 8)
            m["load"] = decode("Present_Load", d[4] | d[5] << 8)
            m["volt"], m["temp"] = d[6] / 10, d[7]
            m["torque"] = bool(torque)
            if not m["torque"]:
                m["goal"] = None
            j = self.cal["joints"].get(i)
            if j and self.cal["phase"] == "recording":
                j["min"], j["max"] = min(j["min"], m["pos"]), max(j["max"], m["pos"])
            elif j and self.cal["phase"] == "posing":  # helps find the middle: range seen so far
                p = m["pos"]
                j["seen_min"] = p if j["seen_min"] is None else min(j["seen_min"], p)
                j["seen_max"] = p if j["seen_max"] is None else max(j["seen_max"], p)
        except BusError:
            m["online"] = False

    def _read_calib_regs(self, i):  # caller holds the lock
        m = self.motors[i]
        m["lo"] = self.bus.read(i, "Min_Position_Limit")
        m["hi"] = self.bus.read(i, "Max_Position_Limit")
        m["offset"] = self.bus.read(i, "Homing_Offset")

    # ---------- helpers ----------
    def _m(self, i):
        m = self.motors.get(int(i))
        if m is None:
            raise CmdError(f"no joint with ID {i}")
        if not m["online"]:
            raise CmdError(f"ID {i} ({m['name']}) is not answering")
        return m

    def _window(self, m):
        lo, hi = m["lo"], m["hi"]
        return (None, None) if lo == hi == 0 else (lo, hi)

    def _calibrating(self, i):
        return self.cal["phase"] in ("posing", "recording") and int(i) in self.cal["ids"]

    def _require_not_calibrating(self, i):
        if self._calibrating(i):
            raise CmdError(f"{JOINTS[int(i)]} is being calibrated; finish or cancel it on the Calibration tab")

    def _send_goal(self, i, pos, dps=None):  # caller holds the lock
        speed = max(1, round((dps or self.speed_dps) * STEPS_PER_DEG))
        g = encode("Goal_Position", int(pos))
        self.bus.write_raw(i, 42, [g & 0xFF, g >> 8 & 0xFF, 0, 0, speed & 0xFF, speed >> 8 & 0xFF])
        self.motors[i]["goal"] = int(pos)

    # ---------- control commands ----------
    def torque(self, i, on):
        m = self._m(i)
        with self.lock:
            if on:
                self._require_not_calibrating(m["id"])
                volt = self.bus.read(m["id"], "Present_Voltage") / 10
                if volt > MAX_VOLTS:
                    raise CmdError(f"bus voltage {volt:.1f} V is above {MAX_VOLTS} V; check the supply")
                pos = self.bus.read(m["id"], "Present_Position")
                lo, hi = self._window(m)
                goal = pos
                if lo is not None and not lo <= pos <= hi:
                    # A joint that went limp often rests on its hard stop, a few steps past the
                    # recorded range. Within EDGE_TOL, ease it back to the edge at the set speed.
                    if lo - EDGE_TOL <= pos <= hi + EDGE_TOL:
                        goal = max(lo, min(hi, pos))
                    else:
                        raise CmdError(f"{m['name']} is at {pos}, outside its limit window [{lo}, {hi}] by more "
                                       f"than {EDGE_TOL / STEPS_PER_DEG:.0f}°; torque-on would snap it to the edge. "
                                       "Move it back inside by hand, or calibrate it.")
                self.bus.write(m["id"], "Acceleration", ACCEL)
                self._send_goal(m["id"], goal)
                self.bus.write(m["id"], "Torque_Enable", 1)
            else:
                self.bus.write(m["id"], "Torque_Enable", 0)
                m["goal"] = None
            m["torque"] = on

    def goal(self, i, pos, dps=None):
        """dps: speed for this move only (the remote's Cartesian mode paces each joint); default = UI speed."""
        m = self._m(i)
        self._require_not_calibrating(m["id"])
        if not m["torque"]:
            raise CmdError(f"turn torque on for {m['name']} first")
        lo, hi = self._window(m)
        pos = int(pos)
        if lo is not None:
            pos = max(lo, min(hi, pos))
        with self.lock:
            self._send_goal(m["id"], pos, dps)
        return pos

    def jog(self, i, deg):
        m = self._m(i)
        base = m["goal"] if m["goal"] is not None else m["pos"]
        return self.goal(i, base + round(float(deg) * STEPS_PER_DEG))

    def hold_all(self):
        with self.lock:
            for m in self.motors.values():
                if m["online"] and m["torque"]:
                    self._send_goal(m["id"], self.bus.read(m["id"], "Present_Position"))

    def limp_all(self):
        errors = []
        with self.lock:
            for i, m in self.motors.items():
                try:
                    self.bus.write(i, "Torque_Enable", 0)
                    m["torque"], m["goal"] = False, None
                except BusError as e:
                    errors.append(str(e))
        if errors and len(errors) == len(self.motors):
            raise CmdError("no motor acknowledged torque off: " + "; ".join(errors))

    def enable_all(self):
        problems = []
        for i, m in self.motors.items():
            if m["online"] and not m["torque"] and not self._calibrating(i):
                try:
                    self.torque(i, True)
                except (CmdError, BusError) as e:
                    problems.append(f"ID {i}: {e}")
        if problems:
            raise CmdError(" | ".join(problems))

    def set_speed(self, dps):
        lo, hi = SPEED_RANGE_DPS
        self.speed_dps = max(lo, min(hi, float(dps)))
        return self.speed_dps

    # ---------- calibration ----------
    def cal_start(self, robot_id, ids=None):
        """Calibrate all joints (ids=None) or just the given ones. Only the joints being
        calibrated go limp and get reset; the others keep their torque and settings."""
        if self.cal["phase"] in ("posing", "recording"):
            raise CmdError("a calibration is already running")
        robot_id = str(robot_id).strip()
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,40}", robot_id):
            raise CmdError("robot id: use 1-40 letters, digits, _ or - (it becomes the file name)")
        ids = sorted(JOINTS) if not ids else sorted({int(x) for x in ids})
        unknown = [i for i in ids if i not in JOINTS]
        if unknown:
            raise CmdError(f"no joint with ID {unknown[0]}")
        offline = [f"{i} ({JOINTS[i]})" for i in ids if not self.motors[i]["online"]]
        if offline:
            raise CmdError("these motors must answer to calibrate; not answering: " + ", ".join(offline))
        with self.lock:
            self._cal_orig = {}
            for i in ids:
                self._cal_orig[i] = {n: self.bus.read(i, n) for n in
                                     ("Homing_Offset", "Min_Position_Limit", "Max_Position_Limit")}
            for i in ids:
                self.bus.torque_off_unlock(i)
                self.bus.write(i, "Operating_Mode", 0)
                self.bus.write(i, "Homing_Offset", 0)
                self.bus.write(i, "Min_Position_Limit", 0)
                self.bus.write(i, "Max_Position_Limit", MAX_RES)
                self.motors[i].update(torque=False, goal=None, offset=0, lo=0, hi=MAX_RES)
            turned = self._extra_turns(ids)
            if turned:  # put everything back and explain
                for i, regs in self._cal_orig.items():
                    for name, value in regs.items():
                        self.bus.write(i, name, value)
                    self._read_calib_regs(i)
                self._cal_orig = {}
                raise CmdError(self._extra_turns_msg(turned))
        self.cal.update(phase="posing", robot_id=robot_id, ids=ids, saved=None,
                        joints={i: {"offset": None, "raw_mid": None, "min": None, "max": None,
                                    "seen_min": None, "seen_max": None} for i in ids})

    def cal_set_middle(self):
        if self.cal["phase"] != "posing":
            raise CmdError("press Start first")
        with self.lock:
            turned = self._extra_turns(self.cal["ids"])
            if turned:
                raise CmdError(self._extra_turns_msg(turned) + " Press Cancel first.")
            for i in self.cal["ids"]:
                m = self.motors[i]
                raw = self.bus.read(i, "Present_Position")  # offset is 0, so this is raw
                off = max(-2047, min(2047, raw - HALF_TURN))  # Homing_Offset is 11-bit magnitude
                self.bus.write(i, "Homing_Offset", off)
                pos = self.bus.read(i, "Present_Position")
                m["offset"], m["pos"] = off, pos
                self.cal["joints"][i].update(offset=off, raw_mid=raw, min=pos, max=pos)
            self.cal["phase"] = "recording"

    def _extra_turns(self, ids):  # caller holds the lock; offsets are 0 here, so present = raw
        """STS3215s keep counting turns in RAM when a joint passes the sensor's 4095->0
        point, and the count only clears on power-up. A reading outside 0..4095 with
        offset 0 can't be centred (the offset is limited to +/-2047)."""
        out = {}
        for i in ids:
            p = self.bus.read(i, "Present_Position")
            if not 0 <= p <= MAX_RES:
                out[i] = p
        return out

    def _extra_turns_msg(self, turned):
        names = ", ".join(f"{JOINTS[i]} (reads {p})" for i, p in turned.items())
        return (f"{names}: reading beyond one turn (0-4095), so the motor's turn counter has "
                "moved and it can't be centred. Unplug the servo power for 5 seconds and plug it "
                "back in (support the arm first), then start again and move the joint straight to "
                "its middle without sweeping it to the ends first.")

    def cal_reset_ranges(self):
        if self.cal["phase"] != "recording":
            raise CmdError("not recording ranges")
        for i, j in self.cal["joints"].items():
            p = self.motors[i]["pos"]
            j["min"] = j["max"] = p

    def cal_save(self):
        if self.cal["phase"] != "recording":
            raise CmdError("not recording ranges")
        joints = self.cal["joints"]
        short = [JOINTS[i] for i, j in joints.items() if j["max"] - j["min"] < MIN_SPAN]
        if short:
            raise CmdError(f"these joints moved less than {MIN_SPAN} steps (~{MIN_SPAN / STEPS_PER_DEG:.0f}°): "
                           + ", ".join(short) + ". Move each one to both ends.")
        bad = [JOINTS[i] for i, j in joints.items() if j["min"] < 0 or j["max"] > MAX_RES]
        if bad:
            raise CmdError("range left 0-4095 for " + ", ".join(bad) +
                           "; the middle pose was probably off. Cancel and start again.")
        with self.lock:
            for i, j in joints.items():
                self.bus.write(i, "Min_Position_Limit", j["min"])
                self.bus.write(i, "Max_Position_Limit", j["max"])
                self._read_calib_regs(i)
        calib = self._merged_calibration(joints)
        text = json.dumps(calib, indent=4)
        paths = []
        for d in self._calib_dirs():
            d.mkdir(parents=True, exist_ok=True)
            p = d / f"{self.cal['robot_id']}.json"
            p.write_text(text)
            paths.append(str(p))
        self._cal_orig = {}
        self.cal.update(phase="done", saved={"paths": paths, "data": calib, "ids": sorted(joints)})

    def _merged_calibration(self, joints):
        """New results for the joints just calibrated, plus every other joint's entry
        from the existing file (or, if it has none, from its motor's registers when
        those hold a real calibration). Joints never calibrated are left out."""
        existing = {}
        p = self._calib_dirs()[0] / f"{self.cal['robot_id']}.json"
        if p.exists():
            try:
                existing = json.loads(p.read_text())
            except (OSError, ValueError):
                existing = {}
        calib = {}
        for i, name in JOINTS.items():
            m = self.motors[i]
            if i in joints:
                j = joints[i]
                calib[name] = {"id": i, "drive_mode": 0, "homing_offset": j["offset"],
                               "range_min": j["min"], "range_max": j["max"]}
            elif isinstance(existing.get(name), dict):
                calib[name] = existing[name]
            elif m["online"] and (m["lo"], m["hi"]) not in ((0, MAX_RES), (0, 0)):
                calib[name] = {"id": i, "drive_mode": 0, "homing_offset": m["offset"],
                               "range_min": m["lo"], "range_max": m["hi"]}
        return calib

    def cal_cancel(self):
        if self.cal["phase"] in ("posing", "recording") and self._cal_orig:
            with self.lock:
                for i, regs in self._cal_orig.items():
                    self.bus.torque_off_unlock(i)
                    for name, value in regs.items():
                        self.bus.write(i, name, value)
                    self._read_calib_regs(i)
            self._cal_orig = {}
        self.cal.update(phase="idle", ids=[], joints={}, saved=None)

    def cal_close(self):
        if self.cal["phase"] == "done":
            self.cal.update(phase="idle", ids=[], joints={})

    def sim_sweep(self):
        if not self.sim:
            raise CmdError("only available with --sim")
        amp = {i: (URDF_DEG[n][1] - URDF_DEG[n][0]) / 2 * STEPS_PER_DEG for i, n in JOINTS.items()}
        self.bus.sweep(amp)

    # Read-only settings dump, to compare motors (e.g. find one stuck in multi-turn mode).
    REG_NAMES = {5: "ID", 6: "Baud_Rate", 9: "Min_Position_Limit", 11: "Max_Position_Limit",
                 16: "Max_Torque", 18: "Phase", 21: "P", 22: "D", 23: "I", 26: "CW_Deadzone",
                 27: "CCW_Deadzone", 30: "Angular_Resolution", 31: "Homing_Offset",
                 33: "Operating_Mode", 40: "Torque_Enable", 48: "Torque_Limit", 55: "Lock",
                 56: "Present_Position"}
    WORD_REGS = {9, 11, 16, 31, 48, 56}

    def read_settings(self):
        out = {}
        with self.lock:
            for i, m in self.motors.items():
                if not m["online"]:
                    continue
                try:
                    raw = bytes(self.bus.read_raw(i, 0, 40)) + bytes(self.bus.read_raw(i, 40, 30))
                except BusError as e:
                    out[i] = {"error": str(e)}
                    continue
                regs = {}
                for addr, name in self.REG_NAMES.items():
                    v = raw[addr] | (raw[addr + 1] << 8 if addr in self.WORD_REGS else 0)
                    if name in ("Homing_Offset", "Present_Position"):
                        v = decode(name, v)
                    regs[name] = v
                out[i] = {"name": m["name"], "regs": regs, "raw": list(raw)}
        return out

    def _calib_dirs(self):
        """Where calibration JSON goes. The simulator never touches the real LeRobot file."""
        return [REPO_CALIB / "sim"] if self.sim else [lerobot_calib_dir(), REPO_CALIB]

    def _saved_file(self):
        p = self._calib_dirs()[0] / f"{self.cal['robot_id']}.json"
        if not p.exists():
            return {"path": str(p), "exists": False}
        try:
            data = json.loads(p.read_text())
        except (OSError, ValueError):
            data = None
        return {"path": str(p), "exists": True, "modified": p.stat().st_mtime, "data": data}

    def state(self):
        return {"port": self.label, "sim": self.sim, "speed_dps": self.speed_dps,
                "poll": {"age": time.time() - self.last_poll if self.last_poll else None,
                         "hz": round(self.poll_hz, 1), "error": self.poll_error},
                "speed_range": SPEED_RANGE_DPS, "max_volts": MAX_VOLTS,
                "steps_per_deg": STEPS_PER_DEG, "center": CENTER, "min_span": MIN_SPAN,
                "motors": [self.motors[i] for i in sorted(self.motors)],
                "cal": {**self.cal, "file": self._saved_file()},
                "remote": self.remote.state()}


# ---------------- simulated bus (no hardware) ----------------
class SimBus:
    """Behaves like sts_bus.Bus for the registers the UI uses, including the homing
    offset (present = raw - offset) and the goal clamp to the limit window."""

    REGS = {"Torque_Enable": "torque", "Acceleration": "acc", "Lock": "lock", "Operating_Mode": "mode",
            "Homing_Offset": "off", "Min_Position_Limit": "lo", "Max_Position_Limit": "hi"}

    def __init__(self):
        start = {1: 2300, 2: 1900, 3: 2100, 4: 1700, 5: 2500, 6: 2047}
        # leftover offsets like the real arm's (2026-11-23), so present can fall outside 0..4095
        old_off = {1: 208, 2: -1737, 3: -485, 4: -1795, 5: -1625, 6: -1772}
        self.m = {i: {"raw": float(p), "goal_raw": float(p), "speed": 0, "torque": 0, "lo": 0,
                      "hi": MAX_RES, "off": old_off[i], "acc": 0, "lock": 1, "mode": 0, "t": time.time()}
                  for i, p in start.items()}

    def _s(self, sid):
        if sid not in self.m:
            raise BusError(f"no sim motor {sid}")
        s = self.m[sid]
        now = time.time()
        dt, s["t"] = now - s["t"], now
        if s["torque"]:
            err = s["goal_raw"] - s["raw"]
            move = max(1, s["speed"]) * dt
            s["raw"] += max(-move, min(move, err))
        return s

    def ping(self, sid, retries=2):
        return sid in self.m

    def read(self, sid, name, retries=3):
        s = self._s(sid)
        if name == "Present_Position":
            return round(s["raw"]) - s["off"]
        if name == "Present_Voltage":
            return 70
        return s[self.REGS[name]]

    def read_raw(self, sid, addr, n, retries=3):
        s = self._s(sid)
        img = bytearray(70)  # a plausible STS3215 register image
        def put(a, v, size=2):
            img[a] = v & 0xFF
            if size == 2:
                img[a + 1] = v >> 8 & 0xFF
        pos = encode("Present_Position", round(s["raw"]) - s["off"])
        moving = s["torque"] and abs(s["goal_raw"] - s["raw"]) > 1
        put(3, 777); put(5, sid, 1); put(9, s["lo"]); put(11, s["hi"]); put(16, 1000)
        put(21, 32, 1); put(22, 32, 1); put(26, 1, 1); put(27, 1, 1); put(30, 1, 1)
        put(31, encode("Homing_Offset", s["off"])); put(33, s["mode"], 1); put(40, s["torque"], 1)
        put(41, s["acc"], 1); put(48, 1000); put(55, s["lock"], 1); put(56, pos)
        put(58, s["speed"] if moving else 0); put(60, 120 if moving else 30 if s["torque"] else 0)
        put(62, 70, 1); put(63, 27, 1)
        return bytes(img[addr:addr + n])

    def write(self, sid, name, value, retries=3):
        s = self._s(sid)
        s[self.REGS[name]] = int(value)

    def write_raw(self, sid, addr, data, retries=3):
        assert addr == 42
        s = self._s(sid)
        goal = decode("Goal_Position", data[0] | data[1] << 8)
        if not s["lo"] == s["hi"] == 0:
            goal = max(s["lo"], min(s["hi"], goal))  # the real motor clamps too
        s["goal_raw"] = goal + s["off"]
        s["speed"] = data[4] | data[5] << 8

    def torque_off_unlock(self, sid):
        self.write(sid, "Torque_Enable", 0)
        self.write(sid, "Lock", 0)

    def sweep(self, amp, seconds=4.0):
        """Pretend a person moves every limp joint to both ends and back."""
        def run():
            mids = {i: s["raw"] for i, s in self.m.items()}
            t0 = time.time()
            while (t := time.time() - t0) < seconds:
                for i, s in self.m.items():
                    if not s["torque"]:
                        s["raw"] = max(0.0, min(MAX_RES, mids[i] + amp[i] * math.sin(2 * math.pi * t / seconds)))
                time.sleep(0.02)
            for i, s in self.m.items():
                if not s["torque"]:
                    s["raw"] = mids[i]
        threading.Thread(target=run, daemon=True).start()


# ---------------- HTTP ----------------
def make_handler(arm):
    remote = arm.remote
    commands = {
        "remote_enable": lambda b: remote.enable(),
        "remote_disable": lambda b: remote.disable(),
        "remote_mode": lambda b: remote.set_mode(b["id"], b["mode"]),
        "remote_style": lambda b: remote.set_style(b["style"]),
        "remote_scale": lambda b: remote.set_scale(b["scale"]),
        "torque": lambda b: arm.torque(b["id"], bool(b["on"])),
        "goal": lambda b: {"goal": arm.goal(b["id"], b["pos"])},
        "jog": lambda b: {"goal": arm.jog(b["id"], b["deg"])},
        "hold": lambda b: arm.hold_all(),
        "limp": lambda b: arm.limp_all(),
        "enable_all": lambda b: arm.enable_all(),
        "speed": lambda b: {"speed_dps": arm.set_speed(b["dps"])},
        "cal_start": lambda b: arm.cal_start(b.get("robot_id", "my_follower"), b.get("ids")),
        "cal_middle": lambda b: arm.cal_set_middle(),
        "cal_reset": lambda b: arm.cal_reset_ranges(),
        "cal_save": lambda b: arm.cal_save(),
        "cal_cancel": lambda b: arm.cal_cancel(),
        "cal_close": lambda b: arm.cal_close(),
        "sim_sweep": lambda b: arm.sim_sweep(),
    }

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(200, HTML.read_bytes(), "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._send(200, arm.state())
            elif self.path == "/api/settings":  # read-only register dump of every motor
                self._send(200, arm.read_settings())
            else:
                self._send(404, {"ok": False, "error": "not found"})

        def do_POST(self):
            name = self.path.removeprefix("/api/")
            if name not in commands:
                return self._send(404, {"ok": False, "error": f"unknown command {name}"})
            try:
                length = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(length) or b"{}")
                result = commands[name](body) or {}
                self._send(200, {"ok": True, **result})
            except (CmdError, BusError, KeyError, ValueError, TypeError, OSError) as e:
                self._send(400, {"ok": False, "error": str(e)})

    return Handler


def find_port():
    for p in list_ports.comports():
        if p.vid == 0x1A86 or "CH34" in (p.description or ""):
            return p.device
    raise SystemExit("Servo adapter not found (no CH34x USB serial port). Use --port or --sim.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, default: auto-detect the adapter")
    ap.add_argument("--http", type=int, default=8101, help="web port (default 8101)")
    ap.add_argument("--sim", action="store_true", help="simulated arm, no hardware")
    args = ap.parse_args()

    if args.sim:
        bus, label = SimBus(), "SIMULATOR"
    else:
        port = args.port or find_port()
        bus, label = Bus(port), port
    arm = Arm(bus, label, sim=args.sim)
    arm.remote = Remote(arm, CmdError)
    server = ThreadingHTTPServer(("127.0.0.1", args.http), make_handler(arm))
    print(f"SO-101 control UI on http://127.0.0.1:{args.http}  ({label}). Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped. Motors keep their current torque state.")


if __name__ == "__main__":
    main()
