"""PlayStation (DualShock 4) remote control for the SO-101, run inside arm_ui.py.

The pad is read in a background thread (pygame joystick API, optional dependency) and
drives the arm *through the Arm class*, so every UI safety rule still applies: voltage
check on torque-on, goal = present before torque, goals clamped to the limit window,
nothing moves while a calibration runs.

Per-joint modes (so the heavily loaded joints 4-6 don't have to heat up):
  active : the controller drives it (torque on)
  hold   : torque on, held where it is, the controller can't move it
  limp   : torque off (the arm sags under gravity on joints 4 and 5)

Safety: there is no deadman button. Joints only move while a stick/button is pressed, and
stopping the input stops the joint. After "Start remote mode" (or a reconnect) nothing moves
until every stick and trigger has been seen at rest. Losing the pad, or no input for STALE_S
seconds, holds every joint in place. PS = hold all, Options = limp all and remote off.
Gripper: X closes, O opens. Wrist roll: L1 / L2.
"""
import json
import math
import os
import threading
import time
from pathlib import Path

import kinematics as kin

# Needed so the pad keeps working while no pygame window exists / has focus.
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

MODES = ("active", "hold", "limp")
DEFAULT_MODES = {1: "active", 2: "active", 3: "active", 4: "hold", 5: "hold", 6: "hold"}

TICK_HZ = 30
DEADZONE = 0.12
MAX_DPS = 30.0              # joint rate at full stick, before the scale factor
LEAD_DEG = 12.0             # goal may lead the measured position by at most this much
GRIPPER_DPS = 40.0
SCALE_STEPS = (0.25, 0.5, 0.75, 1.0, 1.5)
STALE_S = 0.5               # no pad update for this long -> hold everything
TEMP_WARN, TEMP_CUT = 55, 65  # deg C: stop driving the joint / switch it limp

# Default mapping: DualShock 4 as SDL2 reports it through pygame.joystick on Linux (stick
# right/down = +1, triggers rest at -1). Windows numbers the buttons differently. Use
# "Calibrate controller" on the Remote tab to map this pad exactly; the result is saved in
# calibration/pad.json and replaces these defaults. The D-pad also works as a hat on any pad.
AX = {"lx": 0, "ly": 1, "rx": 3, "ry": 4, "l2": 2, "r2": 5}
BTN = {"x": 0, "o": 1, "ps": 10, "options": 9, "l1": 4, "r1": 5, "up": 11, "down": 12, "left": 13, "right": 14}
PAD_FILE = Path(__file__).with_name("calibration") / "pad.json"
PAD_SIM_FILE = Path(__file__).with_name("calibration") / "sim" / "pad.json"
# Controller calibration wizard: (control, kind, prompt). Sticks right/down and triggers
# pressed must read +1; a control that reads the other way is stored as inverted.
PAD_STEPS = (
    ("lx", "axis", "Push the LEFT stick fully RIGHT and hold it"),
    ("ly", "axis", "Push the LEFT stick fully DOWN (toward you) and hold it"),
    ("rx", "axis", "Push the RIGHT stick fully RIGHT and hold it"),
    ("ry", "axis", "Push the RIGHT stick fully DOWN and hold it"),
    ("l2", "axis", "Squeeze L2 fully and hold it"),
    ("r2", "axis", "Squeeze R2 fully and hold it"),
    ("x", "button", "Press and hold X (cross)"),
    ("o", "button", "Press and hold O (circle)"),
    ("l1", "button", "Press and hold L1"),
    ("r1", "button", "Press and hold R1"),
    ("ps", "button", "Press the PS button"),
    ("options", "button", "Press OPTIONS"),
    ("up", "dpad", "Press D-pad UP"),
    ("down", "dpad", "Press D-pad DOWN"),
    ("left", "dpad", "Press D-pad LEFT"),
    ("right", "dpad", "Press D-pad RIGHT"),
)
PAD_DETECT = 0.7        # an axis must move this far from rest to count
PAD_REST = 0.3          # ...and be back within this of rest before the next step
PAD_REST_WAIT_S = 5.0   # give up waiting for release after this long
PAD_STILL_S = 0.8       # the pad must not change for this long before its resting values are taken
PAD_STILL_TOL = 0.05
# stick direction -> joint direction. Flip a sign here if a joint moves the wrong way.
SIGN = {6: 1.0, 5: -1.0, 4: 1.0, 3: 1.0, 2: 1.0, 1: 1.0}  # 4 flipped 2026-11-25: right stick Y inverted

# Cartesian style. IK_IDS = the joints the IK solves (pan, lift, elbow, wrist_flex).
IK_IDS = (6, 5, 4, 3)
CENTER = 2047
# +1 if the motor's position rises when the URDF angle rises. UNVERIFIED on the real arm:
# with a calibration made like lerobot's (drive_mode 0) it should be +1 for all.
DIR = {6: 1.0, 5: 1.0, 4: 1.0, 3: 1.0, 2: 1.0}  # 5 measured +1 on 2026-11-25 (upper arm leaned 45 deg forward read +49 deg); the 2026-11-23 hand check that said -1 was wrong
CART_MPS = 0.06             # tool speed at full stick, m/s (before the scale factor)
CART_RADPS = 0.8            # tool pitch rate at full stick, rad/s
CART_LEASH = 0.03           # target may lead the tool by at most this (m)
CART_REACH_TOL = 0.005      # residual (m) above which the target is pulled back
CART_PITCH_W = 1.0          # pitch error weight vs position (m) in the IK
PAN_X = 0.0388353           # base axis offset from the URDF base_link origin (kinematics.py, first joint)
CART_R_MIN = 0.10           # closest the gripper counts as being to the base axis when turning the base (m)
CART_PACE = 2.0              # goal speed = this x the joint's commanded rate
CART_MAX_JOINT_DPS = 60.0   # IK step limit per joint, deg/s at scale 1


class PygamePad:
    """Thin wrapper over the first connected pygame joystick. poll() -> connected?"""

    def __init__(self):
        import pygame  # noqa: PLC0415 - optional dependency
        self.pg = pygame
        pygame.display.init()
        pygame.joystick.init()
        self.js, self.name, self.axes, self.buttons, self.hat = None, "", [], [], (0, 0)
        self._last_scan = 0.0

    def poll(self):
        pg = self.pg
        pg.event.pump()
        if self.js is not None and not self.js.get_init():
            self.js = None
        if self.js is None:
            if time.time() - self._last_scan < 1.0:
                return False
            self._last_scan = time.time()
            pg.joystick.quit()
            pg.joystick.init()
            if pg.joystick.get_count() == 0:
                return False
            self.js = pg.joystick.Joystick(0)
            self.js.init()
            self.name = self.js.get_name()
        try:
            self.axes = [self.js.get_axis(i) for i in range(self.js.get_numaxes())]
            self.buttons = [self.js.get_button(i) for i in range(self.js.get_numbuttons())]
            self.hat = tuple(self.js.get_hat(0)) if self.js.get_numhats() else (0, 0)  # D-pad on Linux
        except pg.error:
            self.js = None
            return False
        return True


def _shape(v):
    """Deadzone, rescale, and a squared curve for fine control near the centre."""
    if abs(v) < DEADZONE:
        return 0.0
    v = (abs(v) - DEADZONE) / (1 - DEADZONE) * (1 if v > 0 else -1)
    return v * abs(v)


class Remote:
    def __init__(self, arm, error_cls=RuntimeError, pad_factory=PygamePad, tick_hz=TICK_HZ, cart_check=None):
        self.cart_check, self._cart_target = cart_check, None  # cart_check: test hook for cart_ready()
        self.ik = None  # last Cartesian IK step (target, solution, residual), for the Virtual arm tab
        # drive = "real": the controller moves the motors. "virtual": it moves only a virtual arm
        # (self.vq, motor steps per ID) shown on the Virtual arm tab; nothing is written to the bus.
        self.drive, self.vq = "real", None
        # error_cls: arm_ui.CmdError, passed in because arm_ui runs as __main__ and
        # importing it back here would create a second, different class.
        self.arm, self.err, self.pad_factory, self.dt = arm, error_cls, pad_factory, 1 / tick_hz
        self.enabled = False
        self.style = "joint"
        self.modes = dict(DEFAULT_MODES)
        self.scale_i = 3
        self.pad, self.pad_error = None, None
        self.connected, self.pad_name = False, ""
        self.axes, self.buttons, self.hat = [], [], (0, 0)
        self.ax, self.btn, self.inv = dict(AX), dict(BTN), set()  # the active controller mapping
        self.pad_file = PAD_SIM_FILE if getattr(arm, "sim", False) else PAD_FILE
        self.pad_saved_for = None  # controller name the saved mapping was made with
        self.pcal = {"phase": "idle"}
        self._load_pad_map()
        self.message = ""
        self._last_ok, self._moving, self._prev_btn, self._stale_held = 0.0, {}, {}, False
        self._armed, self._trig_seen = False, set()
        self.lock = threading.RLock()
        threading.Thread(target=self._loop, daemon=True).start()

    # ---------- commands (called from the HTTP thread) ----------
    @property
    def scale(self):
        return SCALE_STEPS[self.scale_i]

    def set_mode(self, i, mode):
        i = int(i)
        if i not in self.modes:
            raise ValueError(f"no joint {i}")
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        if self.style == "cartesian" and mode == "limp" and i in IK_IDS:
            raise self.err("limp joints can't be used with Cartesian control; switch to joint style first")
        with self.lock:
            self.modes[i] = mode
            self._cart_target = None
            if self.enabled and self.drive == "real":
                self._apply_mode(i)

    def set_style(self, style):
        if style not in ("joint", "cartesian"):
            raise ValueError("style must be 'joint' or 'cartesian'")
        if style == "cartesian":
            ok, reason = self.cart_ready()
            if not ok:
                raise self.err("Cartesian control isn't available yet: " + reason)
        self.style, self._cart_target = style, None

    def set_drive(self, drive):
        if drive not in ("real", "virtual"):
            raise ValueError("drive must be 'real' or 'virtual'")
        if self.enabled:
            raise self.err("stop remote mode before switching between the real and the virtual arm")
        self.drive, self._cart_target = drive, None

    def set_scale(self, scale):
        self.scale_i = min(range(len(SCALE_STEPS)), key=lambda k: abs(SCALE_STEPS[k] - float(scale)))

    # ---------- controller calibration ----------
    def _load_pad_map(self):
        try:
            data = json.loads(self.pad_file.read_text())
            ax = {k: int(data["axes"][k]) for k in AX}
            btn = {k: (v if v == "hat" else int(v)) for k, v in ((k, data["buttons"][k]) for k in BTN)}
            inv = {k for k in data.get("inverted", []) if k in AX}
        except (OSError, ValueError, KeyError, TypeError):
            return  # no saved mapping (or an unreadable one): keep the defaults
        self.ax, self.btn, self.inv, self.pad_saved_for = ax, btn, inv, data.get("pad")

    def pad_cal_start(self):
        if self.enabled:
            raise self.err("stop remote mode first (PS / Options would trigger remote actions while you map them)")
        if self.pad_error:
            raise self.err(self.pad_error)
        if not self.connected:
            raise self.err("no controller connected")
        with self.lock:
            self.pcal = {"phase": "rest", "step": 0, "found": {}, "inv": set(), "t": time.time(),
                         "base": list(self.axes), "message": "Let go of everything…"}

    def pad_cal_skip(self):
        with self.lock:
            if self.pcal["phase"] in ("rest", "press", "release"):
                self._pcal_next()

    def pad_cal_cancel(self):
        with self.lock:
            self.pcal = {"phase": "idle"}

    def pad_cal_reset(self):
        """Forget the saved mapping and go back to the built-in defaults."""
        if self.enabled or self.pcal["phase"] in ("rest", "press", "release"):
            raise self.err("stop remote mode and any running controller calibration first")
        with self.lock:
            try:
                self.pad_file.unlink()
            except FileNotFoundError:
                pass
            self.ax, self.btn, self.inv, self.pad_saved_for = dict(AX), dict(BTN), set(), None
            self.pcal = {"phase": "idle"}

    def pad_cal_save(self):
        with self.lock:
            if self.pcal["phase"] != "done":
                raise self.err("finish (or skip through) the steps first")
            ax, btn, inv = self._pcal_result()
            for kind, m in (("axis", ax), ("button", btn)):
                used = [v for v in m.values() if v != "hat"]
                dup = sorted({v for v in used if used.count(v) > 1})
                if dup:
                    names = [k for k, v in m.items() if v in dup]
                    raise self.err(f"two controls share {kind} {dup[0]} ({', '.join(names)}); run it again or skip fewer steps")
            data = {"pad": self.pad_name, "axes": ax, "inverted": sorted(inv), "buttons": btn}
            self.pad_file.parent.mkdir(parents=True, exist_ok=True)
            self.pad_file.write_text(json.dumps(data, indent=4))
            self.ax, self.btn, self.inv, self.pad_saved_for = ax, btn, inv, self.pad_name
            self._armed, self._trig_seen = False, set()
            self.pcal = {"phase": "idle", "message": f"Saved to {self.pad_file}"}

    def _pcal_result(self):
        ax, btn, inv = dict(self.ax), dict(self.btn), set(self.inv)
        for key, kind, _ in PAD_STEPS:
            if key not in self.pcal["found"]:
                continue
            v = self.pcal["found"][key]
            if kind == "axis":
                ax[key] = v
                inv.discard(key)
                if key in self.pcal["inv"]:
                    inv.add(key)
            else:
                btn[key] = v
        return ax, btn, inv

    def _pcal_next(self):  # caller holds self.lock
        p = self.pcal
        p["step"] += 1
        p["t"] = time.time()
        if p["step"] >= len(PAD_STEPS):
            p.update(phase="done", message="All steps done. Check the list, then save.")
        else:
            p.update(phase="press", message="")

    def _pcal_at_rest(self, base):
        axes_ok = all(abs(a - b) < PAD_REST for a, b in zip(self.axes, base))
        return axes_ok and not any(self.buttons) and tuple(self.hat) == (0, 0)

    def _pcal_tick(self):
        """Wizard: wait for rest, then for each control wait for the input that moved."""
        with self.lock:
            p = self.pcal
            if p["phase"] not in ("rest", "press", "release"):
                return
            if not self.connected:
                p["message"] = "Controller lost; reconnect it and press Cancel, then start again."
                return
            if p["phase"] == "rest":  # baseline = the readings once nothing has moved for a moment
                moved = len(p["base"]) != len(self.axes) or any(abs(a - b) > PAD_STILL_TOL for a, b in zip(self.axes, p["base"]))
                if moved or any(self.buttons) or tuple(self.hat) != (0, 0):
                    p.update(base=list(self.axes), t=time.time())
                elif time.time() - p["t"] >= PAD_STILL_S:
                    p.update(phase="press", message="")
                return
            if p["phase"] == "release":
                if self._pcal_at_rest(p["base"]) or time.time() - p["t"] > PAD_REST_WAIT_S:
                    self._pcal_next()
                return
            key, kind, _ = PAD_STEPS[p["step"]]
            kinds = {k: kd for k, kd, _ in PAD_STEPS}
            taken_ax = {v for k, v in p["found"].items() if kinds[k] == "axis"}
            taken_btn = {v for k, v in p["found"].items() if kinds[k] != "axis" and v != "hat"}
            if kind == "axis":
                d = [(abs(a - b), i) for i, (a, b) in enumerate(zip(self.axes, p["base"])) if i not in taken_ax]
                if d and max(d)[0] > PAD_DETECT:
                    i = max(d)[1]
                    p["found"][key] = i
                    if self.axes[i] < p["base"][i]:
                        p["inv"].add(key)
                    p.update(phase="release", t=time.time())
                return
            hx, hy = self.hat
            if kind == "dpad" and {"up": hy > 0, "down": hy < 0, "left": hx < 0, "right": hx > 0}[key]:
                p["found"][key] = "hat"
                p.update(phase="release", t=time.time())
                return
            pressed = [i for i, b in enumerate(self.buttons) if b and i not in taken_btn]
            if pressed:
                p["found"][key] = pressed[0]
                p.update(phase="release", t=time.time())

    def _pcal_state(self):
        with self.lock:
            p = self.pcal
            out = {"phase": p["phase"], "message": p.get("message", ""), "total": len(PAD_STEPS)}
            if p["phase"] in ("rest", "press", "release", "done"):
                step = min(p["step"], len(PAD_STEPS) - 1)
                out.update(step=p["step"], key=PAD_STEPS[step][0], prompt=PAD_STEPS[step][2],
                           found=dict(p["found"]), inverted=sorted(p["inv"]),
                           steps=[{"key": k, "prompt": t} for k, _, t in PAD_STEPS])
            return out

    def enable(self):
        if self.pcal["phase"] in ("rest", "press", "release", "done"):
            raise self.err("finish or cancel the controller calibration first")
        if self.arm.cal["phase"] in ("posing", "recording"):
            raise self.err("a calibration is running; finish or cancel it first")
        if self.pad_error:
            raise self.err(self.pad_error)
        with self.lock:
            self.enabled, self.message, self._armed = True, "", False
            self._cart_target, self._moving = None, {}
            if self.drive == "virtual":  # start the virtual arm where the real one is; touch no motor
                self.vq = {}
                for i, m in self.arm.motors.items():
                    p = m["pos"] if m["pos"] is not None else CENTER
                    self.vq[i] = self._clamp(i, p)
                return
            problems = []
            for i in self.modes:
                try:
                    self._apply_mode(i)
                except Exception as e:  # noqa: BLE001 - report per joint, keep going
                    problems.append(f"ID {i}: {e}")
        if problems:
            self.message = "Some joints could not be set up: " + " | ".join(problems)

    def disable(self):
        with self.lock:
            self.enabled = False
            self._moving.clear()

    def _apply_mode(self, i):  # caller holds self.lock
        m = self.arm.motors[i]
        if not m["online"]:
            return
        if self.modes[i] == "limp":
            if m["torque"]:
                self.arm.bus.write(i, "Torque_Enable", 0)
                m["torque"], m["goal"] = False, None
        elif not m["torque"]:
            self.arm.torque(i, True)

    # ---------- state for the UI ----------
    def state(self):
        cart_ok, cart_why = self.cart_ready()
        return {"cart_ok": cart_ok, "cart_why": cart_why,
                "available": self.pad_error is None, "error": self.pad_error, "enabled": self.enabled,
                "style": self.style, "modes": self.modes, "scale": self.scale, "connected": self.connected,
                "pad": self.pad_name, "armed": self._armed, "message": self.message,
                "axes": [round(a, 2) for a in self.axes], "buttons": [int(b) for b in self.buttons],
                "hat": list(self.hat), "pad_cal": self._pcal_state(),
                "pad_map": {"axes": self.ax, "buttons": self.btn, "inverted": sorted(self.inv),
                            "saved": self.pad_file.exists(), "file": str(self.pad_file),
                            "saved_for": self.pad_saved_for},
                "temp_warn": TEMP_WARN, "temp_cut": TEMP_CUT, "drive": self.drive,
                "virtual": self.vq if self.drive == "virtual" else None,
                "ik": self.ik and {**self.ik, "age": time.time() - self.ik["t"]}}

    # ---------- the loop ----------
    def _loop(self):
        while True:
            t0 = time.time()
            try:
                self._tick()
            except Exception as e:  # noqa: BLE001 - never die; fail safe instead
                self.message = f"remote error: {type(e).__name__}: {e}"
                print(self.message)
                self._safe_hold()
            time.sleep(max(0.0, self.dt - (time.time() - t0)))

    def _tick(self):
        if self.pad is None:
            try:
                self.pad = self.pad_factory()
                self.pad_error = None
            except Exception as e:  # noqa: BLE001 - pygame missing or SDL failure
                self.pad_error = f"gamepad support unavailable ({type(e).__name__}: {e}). pip install pygame"
                time.sleep(2)
                return
        ok = self.pad.poll()
        if ok:
            self.axes, self.buttons, self.pad_name = self.pad.axes, self.pad.buttons, self.pad.name
            self.hat = getattr(self.pad, "hat", (0, 0))
            self._last_ok = time.time()
        self.connected = ok
        self._pcal_tick()
        if not self.enabled:
            self._armed = False
            self._prev_btn = self._edges()
            return

        stale = not ok or time.time() - self._last_ok > STALE_S
        btn = self._edges()
        pressed = {k: btn[k] and not self._prev_btn.get(k) for k in btn}
        self._prev_btn = btn

        if pressed["options"]:
            if self.drive == "virtual":
                self.disable()
                self.message = "Options pressed: remote mode off (virtual arm only, the real arm wasn't touched)."
                return
            self.arm.limp_all()
            self.disable()
            self.message = "Options pressed: all joints limp, remote mode off."
            return
        if pressed["ps"]:
            self._safe_hold()
        if stale:
            if not self._stale_held:  # once, not every tick
                self.message = "Controller lost: holding all joints."
                self._safe_hold()
                self._stale_held = True
            self._armed = False
            return
        self._stale_held = False
        if self.arm.cal["phase"] in ("posing", "recording"):
            return

        if self.drive == "real":
            self._heat_guard()
        if pressed["up"]:
            self.scale_i = min(self.scale_i + 1, len(SCALE_STEPS) - 1)
        if pressed["down"]:
            self.scale_i = max(self.scale_i - 1, 0)

        if not self._armed:  # wait until everything is at rest, so enabling can't cause a jump
            if self._neutral(btn):
                self._armed = True
                self.message = ""
            else:
                self.message = "Release all sticks, triggers and buttons to start."
                return
        self._drive_joints(btn)

    def _neutral(self, btn):
        sticks = all(abs(self._axis(a)) < DEADZONE for a in ("lx", "ly", "rx", "ry"))
        return sticks and not self._trigger("l2") > 0.1 and not (btn["l1"] or btn["x"] or btn["o"])

    def _edges(self):
        b, (hx, hy) = self.buttons, self.hat
        hat = {"up": hy > 0, "down": hy < 0, "left": hx < 0, "right": hx > 0}  # D-pad as a hat
        return {k: (i != "hat" and i < len(b) and bool(b[i])) or hat.get(k, False) for k, i in self.btn.items()}

    def _axis(self, name):
        i = self.ax[name]
        if i >= len(self.axes):
            return 0.0
        return -self.axes[i] if name in self.inv else self.axes[i]

    def _trigger(self, name):  # SDL triggers rest at -1 and go to +1
        # Some drivers report 0 (not -1) until a trigger is first touched; ignore it until
        # it has been seen at rest, or an untouched trigger would read as half pressed.
        if len(self.axes) <= self.ax[name]:
            return 0.0
        v = self._axis(name)
        if v < -0.9:
            self._trig_seen.add(name)
        return (v + 1) / 2 if name in self._trig_seen else 0.0

    def _safe_hold(self):
        """Goal = present for every powered joint; clear 'moving' state."""
        self._moving.clear()
        self._cart_target = None
        if self.drive == "virtual":  # the virtual arm just stops; the real arm isn't touched
            return
        try:
            self.arm.hold_all()
        except Exception as e:  # noqa: BLE001
            self.message = f"hold failed: {e}"

    def _heat_guard(self):
        for i, m in self.arm.motors.items():
            t = m["temp"]
            if t is None or self.modes[i] == "limp" or not m["online"]:
                continue
            if t >= TEMP_CUT:
                if self.style == "cartesian" and i in IK_IDS:
                    self.style = "joint"  # limp joints can't be part of the IK
                self.set_mode(i, "limp")
                self.message = f"{m['name']} hit {t} °C: switched to limp."
            elif t >= TEMP_WARN and self.modes[i] == "active":
                self.set_mode(i, "hold")
                self.message = f"{m['name']} is at {t} °C: no longer driven by the controller."

    # ---------- Cartesian style ----------
    def cart_ready(self):
        """(ok, reason). Cartesian angles are only real joint angles after calibration."""
        arm = self.arm
        if self.cart_check is not None:
            return self.cart_check()
        if self.drive == "virtual":  # the virtual arm needs no motors, but the IK needs a joint to move
            if not [i for i in IK_IDS if self.modes[i] == "active"]:
                return False, "no active joint among IDs 3-6 to move"
            return True, ""
        if not all(m["online"] for m in arm.motors.values()):
            return False, "all six motors must be online"
        if not arm._saved_file()["exists"]:
            return False, "no saved calibration file yet (calibrate first)"
        for i in IK_IDS:
            if (arm.motors[i]["lo"], arm.motors[i]["hi"]) in ((0, 4095), (0, 0)):
                return False, f"{arm.motors[i]['name']} has no calibrated limit window"
        bad = [arm.motors[i]["name"] for i in IK_IDS if self.modes[i] == "limp"]
        if bad:
            return False, "limp joints can't be used with Cartesian control: " + ", ".join(bad)
        if not [i for i in IK_IDS if self.modes[i] == "active"]:
            return False, "no active joint among IDs 3-6 to move"
        return True, ""

    def _q(self, m, i):
        """Joint angle in radians (URDF convention) from the commanded goal, else measured."""
        return DIR[i] * (self._goal_of(i) - CENTER) / self.arm_steps_per_deg() * math.pi / 180

    # The drive helpers: the same control code runs the real motors or the virtual arm.
    def _ready(self, i):
        """Can joint i be driven / read right now?"""
        if self.drive == "virtual":
            return self.vq is not None
        m = self.arm.motors[i]
        return m["online"] and m["torque"] and m["pos"] is not None

    def _pos(self, i):
        return self.vq[i] if self.drive == "virtual" else self.arm.motors[i]["pos"]

    def _goal_of(self, i):
        if self.drive == "virtual":
            return self.vq[i]
        m = self.arm.motors[i]
        return m["goal"] if m["goal"] is not None else m["pos"]

    def _clamp(self, i, p):
        m = self.arm.motors[i]
        return p if m["lo"] == m["hi"] == 0 else max(m["lo"], min(m["hi"], p))

    def _send(self, i, goal, dps=None):
        """Real: Arm.goal (all its safety rules). Virtual: the virtual joint is there at once."""
        if self.drive == "virtual":
            self.vq[i] = self._clamp(i, round(goal))
        elif dps is None:
            self.arm.goal(i, goal)
        else:
            self.arm.goal(i, goal, dps)

    def _limits(self, i):
        """URDF limits narrowed to the motor's limit window, in the URDF angle convention."""
        k = IK_IDS.index(i) if i in IK_IDS else 4
        lo, hi = kin.LIMITS[k]
        m = self.arm.motors[i]
        if (m["lo"], m["hi"]) not in ((0, 4095), (0, 0)):
            a = DIR[i] * (m["lo"] - CENTER) / self.arm_steps_per_deg() * math.pi / 180
            b = DIR[i] * (m["hi"] - CENTER) / self.arm_steps_per_deg() * math.pi / 180
            lo, hi = max(lo, min(a, b)), min(hi, max(a, b))
        return lo, hi

    @staticmethod
    def _reach(f, pan):
        """Signed distance of the tool from the base axis along the arm's plane (heading = -pan);
        negative when the gripper is behind the axis."""
        h = -pan
        return (f[0] - PAN_X) * math.cos(h) + f[1] * math.sin(h)

    def _drive_cartesian(self, btn):
        ok, reason = self.cart_ready()
        if not ok:
            self.message = "Cartesian control not available: " + reason
            self._cart_target = None
            return
        arm, dt, sc = self.arm, self.dt, self.scale
        # left stick = gripper left/right and up/down, right stick Y = forward/back (right X = roll, see _drive_joints)
        vfwd = -_shape(self._axis("ry")) * CART_MPS * sc    # right stick up = away from the base
        vlat = -_shape(self._axis("lx")) * CART_MPS * sc    # stick right = right (URDF +y is left)
        vz = _shape(self._axis("ly")) * CART_MPS * sc       # same sign the user picked for the up/down stick on 2026-11-25
        order = (6, 5, 4, 3, 2)                             # JOINT_ORDER as motor IDs
        pan_free = self.modes[6] == "active"
        arm_free = [k for k in (1, 2, 3) if self.modes[order[k]] == "active"]  # lift, elbow, wrist_flex
        free = ([0] if pan_free else []) + arm_free         # joints that get new goals
        motors = [arm.motors[i] for i in order]
        if not all(self._ready(i) for i in order[:4]):
            return
        q = [self._q(m, i) for m, i in zip(motors, order)]
        if not (vfwd or vlat or vz):
            if self._cart_target is not None:        # sticks released: stop right here
                self._cart_target = None
                for k in free:
                    self._send(order[k], self._pos(order[k]))
            return
        # The base turns the gripper around the base axis (polar), so left/right means the same thing
        # wherever the gripper is, and never reverses or blows up near the axis. The IK then only has
        # to solve reach, height and tool angle in the arm's own vertical plane.
        cur = kin.fk(q)
        r_cur = math.hypot(cur[0] - PAN_X, cur[1])
        s_cur = self._reach(cur, q[0])
        if self._cart_target is None:
            self._cart_target = [s_cur, cur[2], cur[3]]   # reach along the arm's plane (negative = behind the axis), height, tool pitch
        t = self._cart_target
        t[0], t[1] = t[0] + vfwd * dt, t[1] + vz * dt
        t[0] = s_cur + max(-CART_LEASH, min(CART_LEASH, t[0] - s_cur))  # leash: the target only leads a little
        t[1] = cur[2] + max(-CART_LEASH, min(CART_LEASH, t[1] - cur[2]))
        t[2] = cur[3] + max(-0.3, min(0.3, t[2] - cur[3]))
        limits = [self._limits(i) for i in order]
        max_dq = math.radians(CART_MAX_JOINT_DPS) * sc * dt
        new = list(q)
        if pan_free and vlat:   # URDF pan angle = minus the gripper's heading around the axis
            dpan = -vlat / max(r_cur, CART_R_MIN) * dt
            dpan = max(-max_dq, min(max_dq, dpan))
            new[0] = max(limits[0][0], min(limits[0][1], q[0] + dpan))
        heading = -new[0]
        tgt = [PAN_X + t[0] * math.cos(heading), t[0] * math.sin(heading), t[1], t[2]]
        for _ in range(3):
            new, _err = kin.ik_step(new, tgt, arm_free, max_dq=max_dq, limits=limits, pitch_weight=CART_PITCH_W)
        f = kin.fk(new)
        residual = math.dist(f[:3], tgt[:3])
        self.ik = {"t": time.time(), "target": [*tgt[:3], math.degrees(tgt[3])], "tool": [*f[:3], math.degrees(f[3])],
                   "q": {n: math.degrees(a) for n, a in zip(kin.JOINT_ORDER, new)}, "residual": residual,
                   "reached": residual <= CART_REACH_TOL, "free": [kin.JOINT_ORDER[k] for k in free]}
        if residual > CART_REACH_TOL:      # unreachable: don't wind the target up past the limit
            self._cart_target = [self._reach(f, new[0]), f[2], f[3]]
        cap = LEAD_DEG * math.pi / 180
        for k in free:
            i = order[k]
            pos_now = DIR[i] * (self._pos(i) - CENTER) / self.arm_steps_per_deg() * math.pi / 180
            qk = max(pos_now - cap, min(pos_now + cap, new[k]))
            goal = round(CENTER + DIR[i] * qk * 180 / math.pi * self.arm_steps_per_deg())
            # pace each joint by how far it has to go this tick so they arrive together
            prev = self._goal_of(i)
            dps = max(3.0, CART_PACE * abs(goal - prev) / self.arm_steps_per_deg() / dt)
            self._send(i, goal, min(dps, 150.0))

    def _drive_joints(self, btn):
        raw = {2: (1.0 if btn["l1"] else 0.0) - _shape(self._trigger("l2")),   # L1 / L2 = roll
               1: (1.0 if btn["o"] else 0.0) - (1.0 if btn["x"] else 0.0)}     # O opens, X closes
        if self.style == "cartesian":
            self._drive_cartesian(btn)
            raw[2] += _shape(self._axis("rx"))  # right stick X tilts the gripper left/right (wrist roll)
        else:
            raw.update({6: _shape(self._axis("lx")), 5: _shape(self._axis("ly")),
                        4: _shape(self._axis("ry")), 3: _shape(self._axis("rx"))})
        for i, v in raw.items():
            if self.modes[i] != "active" or not self._ready(i):
                continue
            v *= SIGN[i]
            if v == 0.0:
                if self._moving.pop(i, False):  # stick released: stop right here
                    self._send(i, self._pos(i))
                continue
            rate = (GRIPPER_DPS if i == 1 else MAX_DPS) * self.scale
            base, pos = self._goal_of(i), self._pos(i)
            cap = round(LEAD_DEG * self.arm_steps_per_deg())
            target = base + v * rate * self.arm_steps_per_deg() * self.dt
            target = max(pos - cap, min(pos + cap, target))
            self._send(i, round(target))
            self._moving[i] = True

    def arm_steps_per_deg(self):
        return 4096 / 360
