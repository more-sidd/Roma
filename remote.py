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
import math
import os
import threading
import time

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

# DualShock 4 as SDL2 reports it through pygame.joystick on Windows. Verified on the real
# pad 2026-11-23 with pad_check.py (stick right/down = +1, triggers rest at -1).
AX = {"lx": 0, "ly": 1, "rx": 2, "ry": 3, "l2": 4, "r2": 5}
BTN = {"x": 0, "o": 1, "ps": 5, "options": 6, "l1": 9, "r1": 10, "up": 11, "down": 12, "left": 13, "right": 14}
# stick direction -> joint direction. Flip a sign here if a joint moves the wrong way.
SIGN = {6: 1.0, 5: -1.0, 4: -1.0, 3: 1.0, 2: 1.0, 1: 1.0}

# Cartesian style. IK_IDS = the joints the IK solves (pan, lift, elbow, wrist_flex).
IK_IDS = (6, 5, 4, 3)
CENTER = 2047
# +1 if the motor's position rises when the URDF angle rises. UNVERIFIED on the real arm:
# with a calibration made like lerobot's (drive_mode 0) it should be +1 for all.
DIR = {6: 1.0, 5: -1.0, 4: 1.0, 3: 1.0, 2: 1.0}  # 5 verified inverted on the real arm 2026-11-23 (dir_check.py)
CART_MPS = 0.06             # tool speed at full stick, m/s (before the scale factor)
CART_RADPS = 0.8            # tool pitch rate at full stick, rad/s
CART_LEASH = 0.03           # target may lead the tool by at most this (m)
CART_REACH_TOL = 0.005      # residual (m) above which the target is pulled back
CART_PITCH_W = 0.3           # pitch error weight vs position (m) in the IK
CART_PACE = 2.0              # goal speed = this x the joint's commanded rate
CART_MAX_JOINT_DPS = 60.0   # IK step limit per joint, deg/s at scale 1


class PygamePad:
    """Thin wrapper over the first connected pygame joystick. poll() -> connected?"""

    def __init__(self):
        import pygame  # noqa: PLC0415 - optional dependency
        self.pg = pygame
        pygame.display.init()
        pygame.joystick.init()
        self.js, self.name, self.axes, self.buttons = None, "", [], []
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
        # error_cls: arm_ui.CmdError, passed in because arm_ui runs as __main__ and
        # importing it back here would create a second, different class.
        self.arm, self.err, self.pad_factory, self.dt = arm, error_cls, pad_factory, 1 / tick_hz
        self.enabled = False
        self.style = "joint"
        self.modes = dict(DEFAULT_MODES)
        self.scale_i = 3
        self.pad, self.pad_error = None, None
        self.connected, self.pad_name = False, ""
        self.axes, self.buttons = [], []
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
            if self.enabled:
                self._apply_mode(i)

    def set_style(self, style):
        if style not in ("joint", "cartesian"):
            raise ValueError("style must be 'joint' or 'cartesian'")
        if style == "cartesian":
            ok, reason = self.cart_ready()
            if not ok:
                raise self.err("Cartesian control isn't available yet: " + reason)
        self.style, self._cart_target = style, None

    def set_scale(self, scale):
        self.scale_i = min(range(len(SCALE_STEPS)), key=lambda k: abs(SCALE_STEPS[k] - float(scale)))

    def enable(self):
        if self.arm.cal["phase"] in ("posing", "recording"):
            raise self.err("a calibration is running; finish or cancel it first")
        if self.pad_error:
            raise self.err(self.pad_error)
        with self.lock:
            self.enabled, self.message, self._armed = True, "", False
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
                "temp_warn": TEMP_WARN, "temp_cut": TEMP_CUT}

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
            self._last_ok = time.time()
        self.connected = ok
        if not self.enabled:
            self._armed = False
            self._prev_btn = self._edges()
            return

        stale = not ok or time.time() - self._last_ok > STALE_S
        btn = self._edges()
        pressed = {k: btn[k] and not self._prev_btn.get(k) for k in btn}
        self._prev_btn = btn

        if pressed["options"]:
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
        b = self.buttons
        return {k: bool(b[i]) if i < len(b) else False for k, i in BTN.items()}

    def _axis(self, name):
        i = AX[name]
        return self.axes[i] if i < len(self.axes) else 0.0

    def _trigger(self, name):  # SDL triggers rest at -1 and go to +1
        # Some drivers report 0 (not -1) until a trigger is first touched; ignore it until
        # it has been seen at rest, or an untouched trigger would read as half pressed.
        if len(self.axes) <= AX[name]:
            return 0.0
        v = self._axis(name)
        if v < -0.9:
            self._trig_seen.add(name)
        return (v + 1) / 2 if name in self._trig_seen else 0.0

    def _safe_hold(self):
        """Goal = present for every powered joint; clear 'moving' state."""
        self._moving.clear()
        self._cart_target = None
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
        p = m["goal"] if m["goal"] is not None else m["pos"]
        return DIR[i] * (p - CENTER) / self.arm_steps_per_deg() * math.pi / 180

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

    def _drive_cartesian(self, btn):
        ok, reason = self.cart_ready()
        if not ok:
            self.message = "Cartesian control not available: " + reason
            self._cart_target = None
            return
        arm, dt, sc = self.arm, self.dt, self.scale
        vx = -_shape(self._axis("ly")) * CART_MPS * sc      # stick up = away from the base
        vy = -_shape(self._axis("lx")) * CART_MPS * sc      # stick right = right (URDF +y is left)
        vz = -_shape(self._axis("ry")) * CART_MPS * sc      # stick up = up
        vp = _shape(self._axis("rx")) * CART_RADPS * sc     # stick right = tool pitches up
        order = (6, 5, 4, 3, 2)                             # JOINT_ORDER as motor IDs
        free = [k for k, i in enumerate(order[:4]) if self.modes[i] == "active"]
        motors = [arm.motors[i] for i in order]
        if any(m["pos"] is None or not m["torque"] for m in motors[:4]):
            return
        q = [self._q(m, i) for m, i in zip(motors, order)]
        if not (vx or vy or vz or vp):
            if self._cart_target is not None:        # sticks released: stop right here
                self._cart_target = None
                for k in free:
                    arm.goal(order[k], motors[k]["pos"])
            return
        cur = kin.fk(q)
        if self._cart_target is None:
            self._cart_target = list(cur)
        t = self._cart_target
        t[0], t[1], t[2], t[3] = t[0] + vx * dt, t[1] + vy * dt, t[2] + vz * dt, t[3] + vp * dt
        for a in range(3):   # leash: the target may only lead the tool a little
            t[a] = cur[a] + max(-CART_LEASH, min(CART_LEASH, t[a] - cur[a]))
        t[3] = cur[3] + max(-0.3, min(0.3, t[3] - cur[3]))
        limits = [self._limits(i) for i in order]
        max_dq = math.radians(CART_MAX_JOINT_DPS) * sc * dt
        new = list(q)
        for _ in range(3):
            new, _err = kin.ik_step(new, t, free, max_dq=max_dq, limits=limits, pitch_weight=CART_PITCH_W)
        f = kin.fk(new)
        residual = math.dist(f[:3], t[:3])
        if residual > CART_REACH_TOL:      # unreachable: don't wind the target up past the limit
            self._cart_target = list(f)
        cap = LEAD_DEG * math.pi / 180
        for k in free:
            i, m = order[k], motors[k]
            pos_now = DIR[i] * (m["pos"] - CENTER) / self.arm_steps_per_deg() * math.pi / 180
            qk = max(pos_now - cap, min(pos_now + cap, new[k]))
            goal = round(CENTER + DIR[i] * qk * 180 / math.pi * self.arm_steps_per_deg())
            # pace each joint by how far it has to go this tick so they arrive together
            prev = m["goal"] if m["goal"] is not None else m["pos"]
            dps = max(3.0, CART_PACE * abs(goal - prev) / self.arm_steps_per_deg() / dt)
            arm.goal(i, goal, min(dps, 150.0))

    def _drive_joints(self, btn):
        raw = {2: (1.0 if btn["l1"] else 0.0) - _shape(self._trigger("l2")),   # L1 / L2 = roll
               1: (1.0 if btn["o"] else 0.0) - (1.0 if btn["x"] else 0.0)}     # O opens, X closes
        if self.style == "cartesian":
            self._drive_cartesian(btn)
        else:
            raw.update({6: _shape(self._axis("lx")), 5: _shape(self._axis("ly")),
                        4: _shape(self._axis("ry")), 3: _shape(self._axis("rx"))})
        for i, v in raw.items():
            m = self.arm.motors[i]
            if self.modes[i] != "active" or not m["online"] or not m["torque"] or m["pos"] is None:
                continue
            v *= SIGN[i]
            if v == 0.0:
                if self._moving.pop(i, False):  # stick released: stop right here
                    self.arm.goal(i, m["pos"])
                continue
            rate = (GRIPPER_DPS if i == 1 else MAX_DPS) * self.scale
            base = m["goal"] if m["goal"] is not None else m["pos"]
            cap = round(LEAD_DEG * self.arm_steps_per_deg())
            target = base + v * rate * self.arm_steps_per_deg() * self.dt
            target = max(m["pos"] - cap, min(m["pos"] + cap, target))
            self.arm.goal(i, round(target))
            self._moving[i] = True

    def arm_steps_per_deg(self):
        return 4096 / 360
